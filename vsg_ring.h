/** @file vsg_ring.h
 ** @brief AVPacket ring operation
 ** @author Zhiwei Zeng
 ** @date 2018.04.11
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _VSG_RING_H_
#define _VSG_RING_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <pthread.h>

#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"

#define RING_BUF_NUM 16
#define RING_BUF_CACHE_MIN_QUEUE 3
#define I_FRAME_INTERVAL   12

typedef struct {
	AVPacket *packet;
} ring_buf_t;

typedef struct {
	ring_buf_t picture[RING_BUF_NUM];
	int in;
	int out;
	int count;
	pthread_mutex_t mutex;
} ring_zone_t;

int ring_put_picture_packet(ring_zone_t *ring);
int ring_get_picture_packet(ring_zone_t *ring);

#ifdef __cplusplus
}
#endif

#endif
