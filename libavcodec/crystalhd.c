/*
 * - CrystalHD decoder module -
 *
 * Copyright(C) 2010 Philip Langdale <ffmpeg.philipl@overt.org>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*****************************************************************************
 * Includes
 ****************************************************************************/

#define _XOPEN_SOURCE 600
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <libcrystalhd/bc_dts_types.h>
#include <libcrystalhd/bc_dts_defs.h>
#include <libcrystalhd/libcrystalhd_if.h>

#include "avcodec.h"
#include "libavutil/intreadwrite.h"

/* Timeout parameter passed to DtsProcOutput() in us */
#define OUTPUT_PROC_TIMEOUT 50
/* Step between fake timestamps passed to hardware in units of 100ns */
#define TIMESTAMP_UNIT 100000


/*****************************************************************************
 * Module private data
 ****************************************************************************/

typedef enum {
    RET_ERROR = -1,
    RET_OK = 0,
    RET_COPY_AGAIN = 1,
    RET_SKIP_NEXT_COPY = 2,
} CopyRet;

/* Need typedef to allow next pointer. */
typedef struct OpaqueList OpaqueList;

struct OpaqueList {
    OpaqueList *next;
    uint64_t fake_timestamp;
    uint64_t reordered_opaque;
};

typedef struct {
    AVCodecContext *avctx;
    AVFrame pic;
    HANDLE dev;

    uint8_t *sps_pps_buf;
    uint32_t sps_pps_size;
    uint8_t nal_length_size;
    uint8_t is_nal;
    uint8_t skip_next_output;

    uint64_t last_picture;

    OpaqueList *head;
    OpaqueList *tail;
} CHDContext;


/*****************************************************************************
 * Helper functions
 ****************************************************************************/

static inline uint8_t id2subtype(CHDContext *priv, enum CodecID id)
{
    switch (id) {
    case CODEC_ID_MPEG4:
        return BC_MSUBTYPE_DIVX;
    case CODEC_ID_MSMPEG4V3:
        return BC_MSUBTYPE_DIVX311;
    case CODEC_ID_MPEG1VIDEO:
        return BC_MSUBTYPE_MPEG1VIDEO;
    case CODEC_ID_MPEG2VIDEO:
        return BC_MSUBTYPE_MPEG2VIDEO;
    case CODEC_ID_VC1:
        return BC_MSUBTYPE_VC1;
    case CODEC_ID_WMV3:
        return BC_MSUBTYPE_WMV3;
    case CODEC_ID_H264:
        return priv->is_nal ? BC_MSUBTYPE_AVC1 : BC_MSUBTYPE_H264;
    default:
        return BC_MSUBTYPE_INVALID;
    }
}

static inline void print_frame_info(CHDContext *priv, BC_DTS_PROC_OUT *output)
{
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tYBuffSz: %u\n", output->YbuffSz);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tYBuffDoneSz: %u\n",
           output->YBuffDoneSz);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tUVBuffDoneSz: %u\n",
           output->UVBuffDoneSz);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tTimestamp: %lu\n",
           output->PicInfo.timeStamp);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tPicture Number: %u\n",
           output->PicInfo.picture_number);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tWidth: %u\n",
           output->PicInfo.width);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tHeight: %u\n",
           output->PicInfo.height);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tChroma: 0x%03x\n",
           output->PicInfo.chroma_format);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tPulldown: %u\n",
           output->PicInfo.pulldown);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tFlags: 0x%08x\n",
           output->PicInfo.flags);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tFrame Rate/Res: %u\n",
           output->PicInfo.frame_rate);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tAspect Ratio: %u\n",
           output->PicInfo.aspect_ratio);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tColor Primaries: %u\n",
           output->PicInfo.colour_primaries);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tMetaData: %u\n",
           output->PicInfo.picture_meta_payload);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tSession Number: %u\n",
           output->PicInfo.sess_num);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tycom: %u\n",
           output->PicInfo.ycom);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tCustom Aspect: %u\n",
           output->PicInfo.custom_aspect_ratio_width_height);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tFrames to Drop: %u\n",
           output->PicInfo.n_drop);
    av_log(priv->avctx, AV_LOG_VERBOSE, "\tH264 Valid Fields: 0x%08x\n",
           output->PicInfo.other.h264.valid);
}

static inline void memcpy_pic(void *dst, const void *src,
                              int bytesPerLine, int height,
                              int dstStride, int srcStride)
{
    int i;
    for (i = 0; i < height; i++) {
        memcpy(dst, src, bytesPerLine);
        src = (const uint8_t*)src + srcStride;
        dst = (uint8_t*)dst + dstStride;
    }
}


/*****************************************************************************
 * OpaqueList functions
 ****************************************************************************/

static uint64_t opaque_list_push(CHDContext *priv, uint64_t reordered_opaque)
{
    OpaqueList *newNode = av_mallocz(sizeof (OpaqueList));
    if (!newNode) {
        av_log(priv->avctx, AV_LOG_ERROR,
               "Unable to allocate new node in OpaqueList.\n");
        return 0;
    }
    if (priv->head == NULL) {
        newNode->fake_timestamp = TIMESTAMP_UNIT;
        priv->head              = newNode;
    } else {
        newNode->fake_timestamp = priv->tail->fake_timestamp + TIMESTAMP_UNIT;
        priv->tail->next        = newNode;
    }
    priv->tail = newNode;
    newNode->reordered_opaque = reordered_opaque;

    return newNode->fake_timestamp;
}

/*
 * The OpaqueList is built in decode order, while elements will be removed
 * in presentation order. If frames are reordered, this means we must be
 * able to remove elements that are not the first element.
 */
static uint64_t opaque_list_pop(CHDContext *priv, uint64_t fake_timestamp)
{
    OpaqueList *node = priv->head;

    if (priv->head == NULL) {
        av_log(priv->avctx, AV_LOG_ERROR,
               "CrystalHD: Attempted to query non-existent timestamps.\n");
        return AV_NOPTS_VALUE;
    }

    /*
     * The first element is special-cased because we have to manipulate
     * the head pointer rather than the previous element in the list.
     */
    if (priv->head->fake_timestamp == fake_timestamp) {
        uint64_t reordered_opaque = node->reordered_opaque;
        priv->head = node->next;
        av_free(node);

        if (priv->head->next == NULL)
            priv->tail = priv->head;

        return reordered_opaque;
    }

    /*
     * The list is processed at arm's length so that we have the
     * previous element available to rewrite its next pointer.
     */
    while (node->next) {
        OpaqueList *next = node->next;
        if (next->fake_timestamp == fake_timestamp) {
            uint64_t reordered_opaque = next->reordered_opaque;
            node->next = next->next;
            av_free(next);

            if (node->next == NULL)
               priv->tail = node;

            return reordered_opaque;
        } else {
            node = next;
        }
    }

    av_log(priv->avctx, AV_LOG_VERBOSE,
           "CrystalHD: Couldn't match fake_timestamp.\n");
    return AV_NOPTS_VALUE;
}


/*****************************************************************************
 * Video decoder API function definitions
 ****************************************************************************/

static void flush(AVCodecContext *avctx)
{
    CHDContext *priv = avctx->priv_data;

    avctx->has_b_frames    = 0;
    priv->last_picture     = 2;
    priv->skip_next_output = 0;
    DtsFlushInput(priv->dev, 4);
}


static av_cold int uninit(AVCodecContext *avctx)
{
    CHDContext *priv = avctx->priv_data;
    HANDLE device;

    if (!priv)
        return 0;

    device = priv->dev;
    DtsStopDecoder(device);
    DtsCloseDecoder(device);
    DtsDeviceClose(device);

    av_free(priv->sps_pps_buf);

    if (priv->pic.data[0])
        avctx->release_buffer(avctx, &priv->pic);

    if (priv->head) {
       OpaqueList *node = priv->head;
       while (node) {
          OpaqueList *next = node->next;
          av_free(node);
          node = next;
       }
    }

    return 0;
}


static av_cold int init(AVCodecContext *avctx)
{
    CHDContext* priv;
    BC_STATUS ret;
    BC_INPUT_FORMAT format = {
        .FGTEnable   = FALSE,
        .Progressive = TRUE,
        .OptFlags    = 0x80000000 | vdecFrameRate59_94 | 0x40,
        .width       = avctx->width,
        .height      = avctx->height,
    };

    uint8_t subtype;

    uint32_t mode = DTS_PLAYBACK_MODE |
                    DTS_LOAD_FILE_PLAY_FW |
                    DTS_SKIP_TX_CHK_CPB |
                    DTS_PLAYBACK_DROP_RPT_MODE |
                    DTS_SINGLE_THREADED_MODE |
                    DTS_DFLT_RESOLUTION(vdecRESOLUTION_1080p23_976);

    av_log(avctx, AV_LOG_VERBOSE, "CrystalHD Init for %s\n",
           avctx->codec->name);

    avctx->pix_fmt = PIX_FMT_YUYV422;

    /* Initialize the library */
    priv               = avctx->priv_data;
    priv->avctx        = avctx;
    priv->is_nal       = avctx->extradata_size > 0 && *(avctx->extradata) == 1;
    priv->last_picture = 2;

    subtype = id2subtype(priv, avctx->codec->id);
    switch (subtype) {
    case BC_MSUBTYPE_AVC1:
        {
            uint8_t *dummy_p;
            int dummy_int;
            AVBitStreamFilterContext *bsfc;

            uint32_t orig_data_size = avctx->extradata_size;
            uint8_t *orig_data = av_malloc(orig_data_size);
            if (!orig_data) {
                av_log(avctx, AV_LOG_ERROR,
                       "Failed to allocate copy of extradata\n");
                return AVERROR(ENOMEM);
            }
            memcpy(orig_data, avctx->extradata, orig_data_size);


            bsfc= av_bitstream_filter_init("h264_mp4toannexb");
            if (!bsfc) {
                av_log(avctx, AV_LOG_ERROR,
                       "Cannot open the h264_mp4toannexb BSF!\n");
                return AVERROR_BSF_NOT_FOUND;
            }
            av_bitstream_filter_filter(bsfc, avctx, NULL, &dummy_p,
                                       &dummy_int, NULL, 0, 0);
            av_bitstream_filter_close(bsfc);

            priv->sps_pps_buf     = avctx->extradata;
            priv->sps_pps_size    = avctx->extradata_size;
            avctx->extradata      = orig_data;
            avctx->extradata_size = orig_data_size;

            format.pMetaData   = priv->sps_pps_buf;
            format.metaDataSz  = priv->sps_pps_size;
            format.startCodeSz = (avctx->extradata[4] & 0x03) + 1;
        }
        break;
    case BC_MSUBTYPE_H264:
        format.startCodeSz = 4;
        // Fall-through
    case BC_MSUBTYPE_VC1:
    case BC_MSUBTYPE_WVC1:
    case BC_MSUBTYPE_WMV3:
    case BC_MSUBTYPE_WMVA:
    case BC_MSUBTYPE_MPEG1VIDEO:
    case BC_MSUBTYPE_MPEG2VIDEO:
    case BC_MSUBTYPE_DIVX:
    case BC_MSUBTYPE_DIVX311:
        format.pMetaData  = avctx->extradata;
        format.metaDataSz = avctx->extradata_size;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "CrystalHD: Unknown codec name\n");
        return -1;
    }
    format.mSubtype = subtype;

    /* Get a decoder instance */
    av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: starting up\n");
    // Initialize the Link and Decoder devices
    ret = DtsDeviceOpen(&priv->dev, mode);
    if (ret != BC_STS_SUCCESS) {
        av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: DtsDeviceOpen failed\n");
        uninit(avctx);
        return -1;
    }

    ret = DtsSetInputFormat(priv->dev, &format);
    if (ret != BC_STS_SUCCESS) {
        av_log(avctx, AV_LOG_ERROR, "CrystalHD: SetInputFormat failed\n");
        uninit(avctx);
        return -1;
    }

    ret = DtsOpenDecoder(priv->dev, BC_STREAM_TYPE_ES);
    if (ret != BC_STS_SUCCESS) {
        av_log(avctx, AV_LOG_ERROR, "CrystalHD: DtsOpenDecoder failed\n");
        uninit(avctx);
        return -1;
    }

    ret = DtsSetColorSpace(priv->dev, OUTPUT_MODE422_YUY2);
    if (ret != BC_STS_SUCCESS) {
        av_log(avctx, AV_LOG_ERROR, "CrystalHD: DtsSetColorSpace failed\n");
        uninit(avctx);
        return -1;
    }
    ret = DtsStartDecoder(priv->dev);
    if (ret != BC_STS_SUCCESS) {
        av_log(avctx, AV_LOG_ERROR, "CrystalHD: DtsStartDecoder failed\n");
        uninit(avctx);
        return -1;
    }
    ret = DtsStartCapture(priv->dev);
    if (ret != BC_STS_SUCCESS) {
        av_log(avctx, AV_LOG_ERROR, "CrystalHD: DtsStartCapture failed\n");
        uninit(avctx);
        return -1;
    }

    av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: Init complete.\n");

    return 0;
}


static inline CopyRet copy_frame(AVCodecContext *avctx,
                                 BC_DTS_PROC_OUT *output,
                                 void *data, int *data_size,
                                 uint8_t second_field)
{
    BC_STATUS ret;
    BC_DTS_STATUS decoder_status;
    uint8_t next_frame_same;
    uint8_t interlaced;
    uint8_t need_second_field;

    CHDContext *priv = avctx->priv_data;

    uint8_t bottom_field = (output->PicInfo.flags & VDEC_FLAG_BOTTOMFIELD) ==
                           VDEC_FLAG_BOTTOMFIELD;
    uint8_t bottom_first = output->PicInfo.flags & VDEC_FLAG_BOTTOM_FIRST;

    int width    = output->PicInfo.width * 2; // 16bits per pixel
    int height   = output->PicInfo.height;
    uint8_t *src = output->Ybuff;
    uint8_t *dst;
    int dStride;

    ret = DtsGetDriverStatus(priv->dev, &decoder_status);
    if (ret != BC_STS_SUCCESS) {
        av_log(avctx, AV_LOG_ERROR,
               "CrystalHD: GetDriverStatus failed: %u\n", ret);
       return RET_ERROR;
    }

    next_frame_same  = output->PicInfo.picture_number ==
                       (decoder_status.picNumFlags & ~0x40000000);
    interlaced        = ((output->PicInfo.flags & VDEC_FLAG_INTERLACED_SRC) &&
                         !(output->PicInfo.flags & VDEC_FLAG_UNKNOWN_SRC)) ||
                        next_frame_same || bottom_field || second_field;
    need_second_field = interlaced &&
                        ((!bottom_field && !bottom_first) ||
                         (bottom_field && bottom_first));

    av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: next_frame_same: %u | %u | %u\n",
           next_frame_same, output->PicInfo.picture_number,
           decoder_status.picNumFlags & ~0x40000000);

    priv->pic.buffer_hints = FF_BUFFER_HINTS_VALID | FF_BUFFER_HINTS_PRESERVE |
                             FF_BUFFER_HINTS_REUSABLE;
    if(avctx->reget_buffer(avctx, &priv->pic) < 0) {
        av_log(avctx, AV_LOG_ERROR, "reget_buffer() failed\n");
        return RET_ERROR;
    }

    dStride = priv->pic.linesize[0];
    dst     = priv->pic.data[0];

    av_log(priv->avctx, AV_LOG_VERBOSE, "CrystalHD: Copying out frame\n");

    if (interlaced) {
        int dY = 0;
        int sY = 0;

        height /= 2;
        if (bottom_field) {
            av_log(priv->avctx, AV_LOG_VERBOSE, "Interlaced: bottom field\n");
            dY = 1;
        } else {
            av_log(priv->avctx, AV_LOG_VERBOSE, "Interlaced: top field\n");
            dY = 0;
        }

        for (sY = 0; sY < height; dY++, sY++) {
            memcpy(&(dst[dY * dStride]), &(src[sY * width]), width);
            if (interlaced)
                dY++;
        }
    } else {
        memcpy_pic(dst, src, width, height, dStride, width);
    }

    priv->pic.interlaced_frame = interlaced;
    if (interlaced)
        priv->pic.top_field_first = !bottom_first;

    if (output->PicInfo.timeStamp != 0) {
        priv->pic.reordered_opaque =
            opaque_list_pop(priv, output->PicInfo.timeStamp);
        av_log(avctx, AV_LOG_VERBOSE, "output \"pts\": %lu\n",
               priv->pic.reordered_opaque);
    }

    if (!need_second_field) {
        *data_size       = sizeof(AVFrame);
        *(AVFrame *)data = priv->pic;
    }

    /*
     * There's no way to detect this case right now. You have to break
     * normal MBAFF and/or PAFF handling to do it.
     */
    if (0 && output->PicInfo.flags & VDEC_FLAG_UNKNOWN_SRC) {
        av_log(priv->avctx, AV_LOG_VERBOSE, "Fieldpair from two packets.\n");
        return RET_SKIP_NEXT_COPY;
    }

    return RET_OK;
}


static inline CopyRet receive_frame(AVCodecContext *avctx,
                                    void *data, int *data_size,
                                    uint8_t second_field)
{
    BC_STATUS ret;
    BC_DTS_PROC_OUT output = {
        .PicInfo.width  = avctx->width,
        .PicInfo.height = avctx->height,
    };
    CHDContext *priv = avctx->priv_data;
    HANDLE dev       = priv->dev;

    *data_size = 0;

    // Request decoded data from the driver
    ret = DtsProcOutputNoCopy(dev, OUTPUT_PROC_TIMEOUT, &output);
    if (ret == BC_STS_FMT_CHANGE) {
        av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: Initial format change\n");
        avctx->width  = output.PicInfo.width;
        avctx->height = output.PicInfo.height;
        if (output.PicInfo.height == 1088)
            avctx->height = 1080;
        return RET_COPY_AGAIN;
    } else if (ret == BC_STS_SUCCESS) {
        int copy_ret = -1;
        if (output.PoutFlags & BC_POUT_FLAGS_PIB_VALID) {
            if (output.PicInfo.timeStamp == 0) {
                av_log(avctx, AV_LOG_VERBOSE,
                       "CrystalHD: Not returning packed frame twice.\n");
                priv->last_picture++;
                DtsReleaseOutputBuffs(dev, NULL, FALSE);
                return RET_COPY_AGAIN;
            }

            print_frame_info(priv, &output);

            if (priv->last_picture + 1 < output.PicInfo.picture_number) {
                av_log(avctx, AV_LOG_WARNING,
                       "CrystalHD: Picture Number discontinuity\n");
                /*
                 * Have we lost frames? If so, we need to shrink the
                 * pipeline length appropriately.
                 */
            }

            copy_ret = copy_frame(avctx, &output, data, data_size, second_field);
            if (*data_size > 0) {
                avctx->has_b_frames--;
                priv->last_picture++;
                av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: Pipeline length: %u\n",
                       avctx->has_b_frames);
            }
        } else {
            /*
             * An invalid frame has been consumed.
             */
            av_log(avctx, AV_LOG_ERROR, "CrystalHD: ProcOutput succeeded with "
                                        "invalid PIB\n");
            avctx->has_b_frames--;
            copy_ret = RET_OK;
        }
        DtsReleaseOutputBuffs(dev, NULL, FALSE);

        return copy_ret;
    } else if (ret == BC_STS_BUSY) {
        return RET_COPY_AGAIN;
    } else {
        av_log(avctx, AV_LOG_ERROR, "CrystalHD: ProcOutput failed %d\n", ret);
        return RET_ERROR;
    }
}


static int decode(AVCodecContext *avctx, void *data, int *data_size, AVPacket *avpkt)
{
    BC_STATUS ret;
    BC_DTS_STATUS decoder_status;
    CopyRet rec_ret;
    CHDContext *priv   = avctx->priv_data;
    HANDLE dev         = priv->dev;
    uint8_t input_full = 0;
    int len            = avpkt->size;

    av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: decode_frame\n");

    do {
        if (len) {
            int32_t tx_free = (int32_t)DtsTxFreeSize(dev);
            if (len < tx_free - 1024) {
                /*
                 * Despite being notionally opaque, either libcrystalhd or
                 * the hardware itself will mangle pts values that are too
                 * small or too large. The docs claim it should be in uints
                 * of 100ns. Given that we're nominally dealing with a black
                 * box on both sides, any transform we do has no guarantee of
                 * avoiding mangling but, empirically, scalling as if the
                 * reorded_opaque value is in ms seems to work.
                 */
                uint64_t pts = opaque_list_push(priv, avctx->reordered_opaque);
                if (!pts) {
                    return AVERROR(ENOMEM);
                }
                av_log(priv->avctx, AV_LOG_VERBOSE, "input \"pts\": %lu\n",
                       pts);
                ret = DtsProcInput(dev, avpkt->data, len, pts, 0);
                if (ret == BC_STS_BUSY) {
                    av_log(avctx, AV_LOG_WARNING,
                           "CrystalHD: ProcInput returned busy\n");
                    usleep(10000);
                    return AVERROR(EBUSY);
                } else if (ret != BC_STS_SUCCESS) {
                    av_log(avctx, AV_LOG_ERROR,
                           "CrystalHD: ProcInput failed: %u\n", ret);
                    return -1;
                }
                len        = 0; // We don't want to try and resubmit the input
                input_full = 0;
                avctx->has_b_frames++;
            } else {
                av_log(avctx, AV_LOG_WARNING, "CrystalHD: Input buffer full\n");
                input_full = 1;
            }
        } else {
            av_log(avctx, AV_LOG_INFO,
                   "CrystalHD: No more input data\n");
        }

        ret = DtsGetDriverStatus(dev, &decoder_status);
        if (ret != BC_STS_SUCCESS) {
            av_log(avctx, AV_LOG_ERROR,
                   "CrystalHD: GetDriverStatus failed\n");
            return -1;
        }
        /*
         * No frames ready. Don't try to extract.
         */
        if (decoder_status.ReadyListCount == 0) {
            usleep(50000);
        } else {
            break;
        }
    } while (input_full == 1);
    if (decoder_status.ReadyListCount == 0) {
        av_log(avctx, AV_LOG_INFO,
               "CrystalHD: No frames ready. Returning\n");
        return avpkt->size;
    }

    if (priv->skip_next_output) {
        av_log(avctx, AV_LOG_VERBOSE,
               "CrystalHD: Skipping next output.\n");
        priv->skip_next_output = 0;
        avctx->has_b_frames--;
        return avpkt->size;
    }

    rec_ret = receive_frame(avctx, data, data_size, 0);
    if (rec_ret == 0 && *data_size == 0) {
        /*
         * XXX: There's more than just mpeg2 vs h.264 interlaced content, and
         * I don't know which category each of those will fall in to.
         */
        if (avctx->codec->id == CODEC_ID_H264) {
            /*
             * This case is for when the encoded fields are stored separately
             * and we get a separate avpkt for each one. To keep the pipeline
             * stable, we should return nothing and wait for the next time
             * round to grab the second field.
             * H.264 PAFF is an example of this.
             */
            av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: Returning after first field.\n");
            avctx->has_b_frames--;
        } else {
            /*
             * This case is for when the encoded fields are stored in a single
             * avpkt but the hardware returns then separately. Unless we grab
             * the second field before returning, we'll slip another frame in
             * the pipeline and if that happens a lot, we're sunk. So we have
             * to get that second field now.
             * Interlaced mpeg2 is an example of this.
             */
            av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: Trying to get second field.\n");
            while (1) {
                ret = DtsGetDriverStatus(dev, &decoder_status);
                if (ret == BC_STS_SUCCESS && decoder_status.ReadyListCount > 0) {
                    rec_ret = receive_frame(avctx, data, data_size, 1);
                    if ((rec_ret == 0 && *data_size > 0) || rec_ret == BC_STS_BUSY)
                        break;
                }
                usleep(10000);
            }
            av_log(avctx, AV_LOG_VERBOSE, "CrystalHD: Got second field.\n");
        }
    } else if (rec_ret == RET_SKIP_NEXT_COPY) {
        /*
         * Two input packets got turned into a field pair. Gawd.
         */
        av_log(avctx, AV_LOG_VERBOSE, "Don't output on next decode call.\n");
        priv->skip_next_output = 1;
    } else if (rec_ret == RET_COPY_AGAIN) {
        /*
         * This means we got a FMT_CHANGE event and no frame, so go around
         * again to get the frame.
         */
        receive_frame(avctx, data, data_size, 0);
    }
    usleep(10000);
    return avpkt->size;
}


#if CONFIG_H264_CRYSTALHD_DECODER
AVCodec ff_h264_crystalhd_decoder = {
    .name           = "h264_crystalhd",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = CODEC_ID_H264,
    .priv_data_size = sizeof(CHDContext),
    .init           = init,
    .close          = uninit,
    .decode         = decode,
    .capabilities   = CODEC_CAP_DR1 | CODEC_CAP_DELAY,
    .flush          = flush,
    .long_name      = NULL_IF_CONFIG_SMALL("H.264 / AVC / MPEG-4 AVC / MPEG-4 part 10 (CrystalHD acceleration)"),
    .pix_fmts       = (const enum PixelFormat[]){PIX_FMT_YUYV422, PIX_FMT_NONE},
};
#endif

#if CONFIG_MPEG1_CRYSTALHD_DECODER
AVCodec ff_mpeg1_crystalhd_decoder = {
    .name           = "mpeg1_crystalhd",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = CODEC_ID_MPEG1VIDEO,
    .priv_data_size = sizeof(CHDContext),
    .init           = init,
    .close          = uninit,
    .decode         = decode,
    .capabilities   = CODEC_CAP_DR1 | CODEC_CAP_DELAY,
    .flush          = flush,
    .long_name      = NULL_IF_CONFIG_SMALL("MPEG-1 Video (CrystalHD acceleration)"),
    .pix_fmts       = (const enum PixelFormat[]){PIX_FMT_YUYV422, PIX_FMT_NONE},
};
#endif

#if CONFIG_MPEG2_CRYSTALHD_DECODER
AVCodec ff_mpeg2_crystalhd_decoder = {
    .name           = "mpeg2_crystalhd",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = CODEC_ID_MPEG2VIDEO,
    .priv_data_size = sizeof(CHDContext),
    .init           = init,
    .close          = uninit,
    .decode         = decode,
    .capabilities   = CODEC_CAP_DR1 | CODEC_CAP_DELAY,
    .flush          = flush,
    .long_name      = NULL_IF_CONFIG_SMALL("MPEG-2 Video (CrystalHD acceleration)"),
    .pix_fmts       = (const enum PixelFormat[]){PIX_FMT_YUYV422, PIX_FMT_NONE},
};
#endif

#if CONFIG_MPEG4_CRYSTALHD_DECODER
AVCodec ff_mpeg4_crystalhd_decoder = {
    .name           = "mpeg4_crystalhd",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = CODEC_ID_MPEG4,
    .priv_data_size = sizeof(CHDContext),
    .init           = init,
    .close          = uninit,
    .decode         = decode,
    .capabilities   = CODEC_CAP_DR1 | CODEC_CAP_DELAY,
    .flush          = flush,
    .long_name      = NULL_IF_CONFIG_SMALL("MPEG-4 Part 2 (CrystalHD acceleration)"),
    .pix_fmts       = (const enum PixelFormat[]){PIX_FMT_YUYV422, PIX_FMT_NONE},
};
#endif

#if CONFIG_MSMPEG4_CRYSTALHD_DECODER
AVCodec ff_msmpeg4_crystalhd_decoder = {
    .name           = "msmpeg4_crystalhd",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = CODEC_ID_MSMPEG4V3,
    .priv_data_size = sizeof(CHDContext),
    .init           = init,
    .close          = uninit,
    .decode         = decode,
    .capabilities   = CODEC_CAP_DR1 | CODEC_CAP_DELAY,
    .flush          = flush,
    .long_name      = NULL_IF_CONFIG_SMALL("MPEG-4 Part 2 Microsoft variant version 3 (CrystalHD acceleration)"),
    .pix_fmts       = (const enum PixelFormat[]){PIX_FMT_YUYV422, PIX_FMT_NONE},
};
#endif

#if CONFIG_VC1_CRYSTALHD_DECODER
AVCodec ff_vc1_crystalhd_decoder = {
    .name           = "vc1_crystalhd",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = CODEC_ID_VC1,
    .priv_data_size = sizeof(CHDContext),
    .init           = init,
    .close          = uninit,
    .decode         = decode,
    .capabilities   = CODEC_CAP_DR1 | CODEC_CAP_DELAY,
    .flush          = flush,
    .long_name      = NULL_IF_CONFIG_SMALL("SMPTE VC-1 (CrystalHD acceleration)"),
    .pix_fmts       = (const enum PixelFormat[]){PIX_FMT_YUYV422, PIX_FMT_NONE},
};
#endif

#if CONFIG_WMV3_CRYSTALHD_DECODER
AVCodec ff_wmv3_crystalhd_decoder = {
    .name           = "wmv3_crystalhd",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = CODEC_ID_WMV3,
    .priv_data_size = sizeof(CHDContext),
    .init           = init,
    .close          = uninit,
    .decode         = decode,
    .capabilities   = CODEC_CAP_DR1 | CODEC_CAP_DELAY,
    .flush          = flush,
    .long_name      = NULL_IF_CONFIG_SMALL("Windows Media Video 9 (CrystalHD acceleration)"),
    .pix_fmts       = (const enum PixelFormat[]){PIX_FMT_YUYV422, PIX_FMT_NONE},
};
#endif
