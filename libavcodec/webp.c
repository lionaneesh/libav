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
    int size;
} WebPContext;

static int webp_decode_frame(AVCodecContext *avctx, void *data, int *got_frame,
                             AVPacket *avpkt)
{
    AVFrame * const p = data;
    int ret;
    GetByteContext g;
    unsigned int size, chunk_size;
    
    bytestream2_init(&g, avpkt->data, avpkt->size);

    if (bytestream2_get_bytes_left(&g) < 12)
        return AVERROR_INVALIDDATA;

    if (bytestream2_get_le32(&g) != MKTAG('R', 'I', 'F', 'F')) {
        av_log(avctx, AV_LOG_ERROR, "missing RIFF tag\n");
        return AVERROR_INVALIDDATA;
    }
    
    size = bytestream2_get_le32(&g);

    if (bytestream2_get_le32(&g) != MKTAG('W', 'E', 'B', 'P')) {
        av_log(avctx, AV_LOG_ERROR, "missing WEBP tag\n");
        return AVERROR_INVALIDDATA;
    }
    
    while (bytestream2_get_bytes_left(&g) > 0) {
        unsigned int chunk_type = bytestream2_get_le32(&g);
        unsigned int chunk_size = bytestream2_get_le32(&g);
        
        switch (chunk_type) {
            case MKTAG('V', 'P', '8', ' '):
                // decoding goes here
                bytestream2_skip(&g, chunk_size - 4);
                break;
            default :
                bytestream2_skip(&g, chunk_size - 4);
        }
    }
    
    avctx->width   = 800;
    avctx->height  = 600;
    avctx->pix_fmt = AV_PIX_FMT_YUV420P;
    
    if ((ret = ff_get_buffer(avctx, p, 0)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
        return ret;
    }
    
    memset(p->data[0], 0,    p->linesize[0]);
    memset(p->data[1], 100, p->linesize[1]);
    memset(p->data[2], 100, p->linesize[2]);
    
    p->pict_type = AV_PICTURE_TYPE_I;
    *got_frame = 1;
    return avpkt->size;
}

static av_cold int webp_decode_init(AVCodecContext *avctx)
{
    return 0;
}


AVCodec ff_webp_decoder = {
    .name           = "webp",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_WEBP,
    .init           = webp_decode_init,
    .decode         = webp_decode_frame,
    .capabilities   = CODEC_CAP_DR1,
    .priv_data_size = sizeof(WebPContext),
    .long_name      = NULL_IF_CONFIG_SMALL("WebP image"),
};