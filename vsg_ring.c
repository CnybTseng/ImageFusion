/** @file vsg_ring.c - Implementation
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

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "vsg_ring.h"

int ring_put_picture_packet(ring_zone_t *ring)
{
	if( ring->in==(RING_BUF_NUM-1)) {
		ring->in = (ring->in + 1) % RING_BUF_NUM;
		pthread_mutex_lock(&ring->mutex);
		ring->count++;
		pthread_mutex_unlock(&ring->mutex);
		while((ring->count)>0) {
//		printf("***ring->count >RING_BUF_CACHE_MIN_QUEUE****\n");
#ifdef _WIN32
			Sleep(10);
#else
			usleep(10000);
#endif
		}
		return 0;
	}

	ring->in = (ring->in + 1) % RING_BUF_NUM;
	pthread_mutex_lock(&ring->mutex);
	ring->count++;
	pthread_mutex_unlock(&ring->mutex);
	return 0;
}

int ring_get_picture_packet(ring_zone_t *ring)
{
	if((ring->count) <=0) {
		return -1;
	}
	ring->out = (ring->out + 1) % RING_BUF_NUM;
	pthread_mutex_lock(&ring->mutex);
	ring->count--;
	pthread_mutex_unlock(&ring->mutex);

	return 0;
}


