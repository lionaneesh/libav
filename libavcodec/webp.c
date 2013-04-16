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

typedef struct WebPContext {
    VP8Context v;
    int initialized;
} WebPContext;

static int vp8_lossless_decode_frame(AVCodecContext *avctx, AVFrame *p,
                                     int *got_frame, uint8_t *data_start,
                                     int data_size)
{
    int ret;

    avctx->width   = 800;
    avctx->height  = 600;
    avctx->pix_fmt = AV_PIX_FMT_ARGB;

    if ((ret = ff_get_buffer(avctx, p, 0)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
        return ret;
    }

    memset(p->data[0], 0, p->linesize[0] * avctx->height);

    *got_frame = 1;

    return data_size;
}

static int vp8_lossy_decode_frame(AVCodecContext *avctx, AVFrame *p,
                                  int *got_frame, uint8_t *data_start,
                                  int data_size)
{
    WebPContext *s = avctx->priv_data;
    AVPacket pkt;

    if (!s->initialized) {
        vp8_decode_init(avctx);
        s->initialized = 1;
    }

    av_init_packet(&pkt);
    pkt.data = data_start;
    pkt.size = data_size;

    return vp8_decode_frame(avctx, p, got_frame, &pkt);
}

static int webp_decode_frame(AVCodecContext *avctx, void *data, int *got_frame,
                             AVPacket *avpkt)
{
    AVFrame * const p = data;
    int ret = 0;
    GetByteContext g;
    unsigned int chunk_type, chunk_size;

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

    chunk_type = bytestream2_get_le32(&g);
    chunk_size = bytestream2_get_le32(&g);

    if (bytestream2_get_bytes_left(&g) < chunk_size || chunk_size > INT_MAX)
        return AVERROR_INVALIDDATA;

    switch (chunk_type) {
    case MKTAG('V', 'P', '8', ' '):
        ret = vp8_lossy_decode_frame(avctx, p, got_frame,
                                     avpkt->data + bytestream2_tell(&g),
                                     chunk_size);
        if (ret < 0)
            return ret;
        break;
    case MKTAG('V', 'P', '8', 'L'):
        ret = vp8_lossless_decode_frame(avctx, p, got_frame,
                                        avpkt->data + bytestream2_tell(&g),
                                        chunk_size);
        if (ret < 0)
            return ret;
        break;
    case MKTAG('V', 'P', '8', 'X'):
        av_log(avctx, AV_LOG_ERROR, "VP8X is not implemented\n");
        return AVERROR_PATCHWELCOME;
    default:
        av_log(avctx, AV_LOG_ERROR, "invalid WebP image type\n");
        return AVERROR_INVALIDDATA;
    }

    if (*got_frame) {
        p->pict_type = AV_PICTURE_TYPE_I;
        p->key_frame = 1;
    }

    return ret;
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
