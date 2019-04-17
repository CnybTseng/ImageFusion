/** @file vsg_init.c - Implementation
 ** @brief Video stream grabber initialization
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
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include "vsg_init.h"
#include "vsg_ring.h"
#include "vsg_recorder.h"

/* 线程属性初始化 */
void init_thread_attribute(video_pic_capture_context *context)
{
	pthread_attr_init(&context->pattr);
	pthread_attr_setdetachstate(&context->pattr, PTHREAD_CREATE_DETACHED);
}

void init_ring_queue(video_pic_capture_context *context)
{
	context->yuv_frame = av_frame_alloc();
	if(context->yuv_frame == NULL) {
		printf("yuv frame malloc fail!!!\n");
		return ;
	}
	context->save_frame = av_frame_alloc();
	if(context->save_frame == NULL) {
		printf("yuv frame malloc fail!!!\n");
		return ;
	}
	/*	申请所需的空间 */
	if( video_malloc_img_convert_buffer(&context->video,&context->picture_convert) == -1) {
		printf("malloc img convert buffer error!!!\n");
		return ;
	}
	/*循环队列资源申请*/
	context->ring_zone = (ring_zone_t *)calloc(1, sizeof(ring_zone_t)) ;
	if(context->ring_zone == NULL) {
		printf("ring zone calloc fail\n");
		return ;
	}
	context->ring_zone->in=0;
	context->ring_zone->count=0;
	context->ring_zone->out=0;
	int i;
	for(i=0; i<RING_BUF_NUM; i++) {
		context->ring_zone->picture[i].packet = (AVPacket *)av_malloc(sizeof(AVPacket));
		if(context->ring_zone->picture[i].packet == NULL) {
			printf("packet malloc fail!!!\n");
			return ;
		}
		av_init_packet(context->ring_zone->picture[i].packet);
	}

	pthread_mutex_init(&context->ring_zone->mutex,NULL);
	printf("\n*********init_ring_queue is ok************\n");
	return;
}

