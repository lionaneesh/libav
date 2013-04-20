/*
 * WebP (.webp) image decoder
 * Copyright (c) 2013 Aneesh Dogra <aneesh@sugarlabs.org>
 *
 * This file is part of Libav.
 *
 * Libav is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Libav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavcodec/bytestream.h"
#include "avcodec.h"
#include "internal.h"
#include "vp8.h"
#include "get_bits.h"

typedef struct WebP_VP8X {
    // uint8_t rsv;
    uint8_t icc;
    uint8_t alpha;
    uint8_t exif_metadata;
    uint8_t xmp_metadata;
    uint8_t animation;
    // uint8_t rsv2;
    // uint24_t rsv3;
    int width;
    int height;
} WebP_VP8X;

typedef struct WebPContext {
    VP8Context v;
    int initialized;
    GetByteContext alpha_bitstream_g;
    int alpha_bitstream_size;
    WebP_VP8X vp8x;
} WebPContext;

static int vp8_lossless_decode_frame(AVCodecContext *avctx, AVFrame *p,
                                     int *got_frame, uint8_t *data_start,
                                     unsigned int data_size)
{
    int ret;

    avctx->width   = 800;
    avctx->height  = 600;
    avctx->pix_fmt = AV_PIX_FMT_ARGB;

    av_log(avctx, AV_LOG_DEBUG, "VP8 Lossless\n");

    if ((ret = ff_get_buffer(avctx, p, 0)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
        return ret;
    }

    memset(p->data[0], 0, p->linesize[0] * avctx->height);

    *got_frame = 1;
    p->pict_type = AV_PICTURE_TYPE_I;
    p->key_frame = 1;

    return data_size;
}

static int vp8_lossy_decode_frame(AVCodecContext *avctx, AVFrame *p,
                                  int *got_frame, uint8_t *data_start,
                                  unsigned int data_size, WebPContext *s)
{
    AVPacket pkt;
    int ret;

    av_log(avctx, AV_LOG_DEBUG, "VP8 Lossy\n");

    if (!s->initialized) {
        vp8_decode_init(avctx);
        s->initialized = 1;
        if (s->alpha_bitstream_size > 0)
            avctx->pix_fmt = AV_PIX_FMT_YUVA420P;
    }

    if (data_size > INT_MAX) {
        av_log(avctx, AV_LOG_DEBUG, "unsupported chunk size\n");
        return AVERROR_PATCHWELCOME;
    }

    av_init_packet(&pkt);
    pkt.data = data_start;
    pkt.size = data_size;

    ret = vp8_decode_frame(avctx, p, got_frame, &pkt);
    if (s->alpha_bitstream_size > 0) {
        int y;
        for (y = 0; y < s->vp8x.height; y++) {
            bytestream2_get_buffer(&s->alpha_bitstream_g, p->data[3] + p->linesize[3] * y + 1, s->vp8x.width);

        }
    }
    return ret;
}

static int webp_decode_frame(AVCodecContext *avctx, void *data, int *got_frame,
                             AVPacket *avpkt)
{
    AVFrame * const p = data;
    WebPContext *s = avctx->priv_data;
    GetByteContext g;
    int ret;
    unsigned int chunk_type, chunk_size;

    s->vp8x.width  = 0;
    s->vp8x.height = 0;
    *got_frame = 0;
    s->alpha_bitstream_size = 0;
    bytestream2_init(&g, avpkt->data, avpkt->size);

    if (bytestream2_get_bytes_left(&g) < 12)
        return AVERROR_INVALIDDATA;

    if (bytestream2_get_le32(&g) != MKTAG('R', 'I', 'F', 'F')) {
        av_log(avctx, AV_LOG_ERROR, "missing RIFF tag\n");
        return AVERROR_INVALIDDATA;
    }

    chunk_size = bytestream2_get_le32(&g);
    if (bytestream2_get_bytes_left(&g) < chunk_size)
        return AVERROR_INVALIDDATA;

    if (bytestream2_get_le32(&g) != MKTAG('W', 'E', 'B', 'P')) {
        av_log(avctx, AV_LOG_ERROR, "missing WEBP tag\n");
        return AVERROR_INVALIDDATA;
    }

    while (bytestream2_get_bytes_left(&g) > 0) {
        chunk_type = bytestream2_get_le32(&g);
        chunk_size = bytestream2_get_le32(&g);
        if (chunk_size == UINT32_MAX)
            return AVERROR_INVALIDDATA;
        chunk_size += chunk_size & 1;

        if (bytestream2_get_bytes_left(&g) < chunk_size)
            return AVERROR_INVALIDDATA;

        switch (chunk_type) {
        case MKTAG('V', 'P', '8', ' '):
            if (!*got_frame) {
                ret = vp8_lossy_decode_frame(avctx, p, got_frame,
                                             avpkt->data + bytestream2_tell(&g),
                                             chunk_size, s);
                if (ret < 0)
                    return ret;
            }
            bytestream2_skip(&g, chunk_size);
            break;
        case MKTAG('V', 'P', '8', 'L'):
            if (!*got_frame) {
                ret = vp8_lossless_decode_frame(avctx, p, got_frame,
                                                avpkt->data + bytestream2_tell(&g),
                                                chunk_size);
                if (ret < 0)
                    return ret;
            }
            bytestream2_skip(&g, chunk_size);
            break;
        case MKTAG('V', 'P', '8', 'X'): {
                uint8_t info_bits;

                info_bits = bytestream2_get_byte(&g);
                bytestream2_skip(&g, 3);

                //s->vp8x.rsv2          = info_bits >> 0 & 0x1;
                s->vp8x.animation     = info_bits >> 1 & 0x1;
                s->vp8x.xmp_metadata  = info_bits >> 2 & 0x1;
                s->vp8x.exif_metadata = info_bits >> 3 & 0x1;
                s->vp8x.alpha         = info_bits >> 4 & 0x1;
                s->vp8x.icc           = info_bits >> 5 & 0x1;
                //s->vp8x.rsv           = info_bits >> 6 & 0xFF;

                s->vp8x.width  = bytestream2_get_le24(&g) + 1;
                s->vp8x.height = bytestream2_get_le24(&g) + 1;
                break;
        }
        case MKTAG('A', 'L', 'P', 'H'): {
            uint8_t alpha_header_data;
            GetByteContext g_a = g;
            int bitstream_size = s->vp8x.width * s->vp8x.height;
            uint8_t rsv, pre_p, filter_m, compression;

            bytestream2_skip(&g, chunk_size);
            alpha_header_data = bytestream2_get_byte(&g_a);

            rsv         = alpha_header_data >> 3 & 0x03;
            pre_p       = alpha_header_data >> 2 & 0x03;
            filter_m    = alpha_header_data >> 1 & 0x03;
            compression = alpha_header_data >> 0 & 0x03;

            if (pre_p       != 0 ||
                filter_m    != 0 ||
                compression != 0) {
                    av_log(avctx, AV_LOG_VERBOSE, "skipping unsupported ALPHA chunk\n");
                    break;
            }
            s->alpha_bitstream_g    = g_a;
            s->alpha_bitstream_size = bitstream_size;

            av_log(avctx, AV_LOG_WARNING, "chunk_size: %d, bitstream_size: %d, compression: %d, rsv: %d, pre_p: %d, filter_m: %d\n", chunk_size, bitstream_size, compression, rsv, pre_p, filter_m);

            break;
        }
        case MKTAG('I', 'C', 'C', 'P'):
        case MKTAG('A', 'N', 'I', 'M'):
        case MKTAG('A', 'N', 'M', 'F'):
        case MKTAG('E', 'X', 'I', 'F'):
        case MKTAG('X', 'M', 'P', ' '):
            av_log(avctx, AV_LOG_VERBOSE, "skipping unsupported chunk\n");
            bytestream2_skip(&g, chunk_size);
            break;
        default:
            av_log(avctx, AV_LOG_WARNING, "skipping unknown chunk\n");
            bytestream2_skip(&g, chunk_size);
            break;
        }
    }

    if (!*got_frame) {
        av_log(avctx, AV_LOG_ERROR, "image data not found\n");
        return AVERROR_INVALIDDATA;
    }
    return avpkt->size;
}


AVCodec ff_webp_decoder = {
    .name           = "webp",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_WEBP,
    .decode         = webp_decode_frame,
    .capabilities   = CODEC_CAP_DR1,
    .priv_data_size = sizeof(WebPContext),
    .long_name      = NULL_IF_CONFIG_SMALL("WebP image"),
};
