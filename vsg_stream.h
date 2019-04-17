/** @file vsg_stream.h
 ** @brief Video stream processing
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

#ifndef _VSG_STREAM_H_
#define _VSG_STREAM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "vsg_ring.h"

typedef struct {
	int index;
	AVCodecContext	*codec;
} video_info_t;

typedef struct {
	AVFrame *yuv_frame;
	AVFrame *rgb_frame;
	unsigned char *out_buf;
	struct SwsContext *img_convert_ctx;
} video_convert_t;

typedef struct {
	video_info_t video;
	AVFrame *yuv_frame;
	AVFrame *save_frame;
	pthread_attr_t pattr;
	video_convert_t picture_convert;
	ring_zone_t *ring_zone;
	pthread_mutex_t out_mutex;
	AVFormatContext	*ff;
	AVCodecParserContext *pCodecParserCtx;
	AVCodec *rtsp_codec;
	int runing_flag;
	unsigned long long pic_index;
	char *stream;
} video_pic_capture_context;

typedef struct {
	unsigned char *data;
	int height, width;
} rgb_pic;

typedef struct {
	unsigned char *data;
	int height, width;
	AVCodecContext	*codec;
} yuv_pic;

typedef struct {
	unsigned long long last_pic_index;
	video_pic_capture_context *capture_handle;
} video_pic_reader;

int init_video_pic_capture(char *stream);
int start_video_pic_capture(int handle);
void pause_video_pic_capture(int handle);
int resume_video_pic_capture(int handle,char *stream);
void stop_video_pic_capture(int handle);
void free_video_pic_capture(int handle);

video_pic_reader *create_video_pic_reader(int capture_handle);
void free_video_pic_reader(video_pic_reader *reader);

rgb_pic *capture_video_rgb_data(video_pic_reader *reader);
void free_video_rgb_pic(rgb_pic *pic);

yuv_pic *capture_video_yuv_data(video_pic_reader *reader);
void free_video_yuv_pic(yuv_pic *pic);

rgb_pic *yuv_to_rgb_data(video_pic_reader *reader);
rgb_pic *yuv_to_rgb(yuv_pic *pic,int width,int height);

#ifdef __cplusplus
}
#endif

#endif

