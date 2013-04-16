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

#define VP8X_FLAG_ANIMATION     0x2
#define VP8X_FLAG_XMP_METADATA  0x4
#define VP8X_FLAG_EXIF_METADATA 0x8
#define VP8X_FLAG_ALPHA         0x10
#define VP8X_FLAG_ICC           0x20

typedef struct WebPContext {
    VP8Context v;
    int initialized;
    GetByteContext alpha_bitstream_g;
    uint8_t has_alpha;
    int width;
    int height;
} WebPContext;

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
        if (s->has_alpha)
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
    if (s->has_alpha) {
        int y;
        for (y = 0; y < s->height; y++)
            bytestream2_get_buffer(&s->alpha_bitstream_g,
                                   p->data[3] + p->linesize[3] * y,
                                   s->width);
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
    uint8_t vp8x_flags;

    s->width  = 0;
    s->height = 0;
    *got_frame = 0;
    s->has_alpha = 0;
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
            av_log(avctx, AV_LOG_DEBUG, "VP8L not supported\n");
            return AVERROR_PATCHWELCOME;
        case MKTAG('V', 'P', '8', 'X'): {
            vp8x_flags = bytestream2_get_byte(&g);
            bytestream2_skip(&g, 3);
            s->width  = bytestream2_get_le24(&g) + 1;
            s->height = bytestream2_get_le24(&g) + 1;
            break;
        }
        case MKTAG('A', 'L', 'P', 'H'): {
            uint8_t alpha_header_data;
            uint8_t rsv, pre_p, filter_m, compression;

            if (!(vp8x_flags & VP8X_FLAG_ALPHA))
                av_log(avctx, AV_LOG_WARNING, "ALPHA chunk present, but alpha bit "\
                                              "not set in the VP8X header\n");
            s->alpha_bitstream_g = g;
            bytestream2_skip(&g, chunk_size);
            alpha_header_data = bytestream2_get_byte(&s->alpha_bitstream_g);

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
            s->has_alpha = 1;

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
