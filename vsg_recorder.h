/** @file vsg_recorder.h
 ** @brief Video stream recorder
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

#ifndef _VSG_RECORDER_H_
#define _VSG_RECORDER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <time.h>

#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"

#include "vsg_stream.h"

#define MIN(a, b) ((a) > (b) ? (b) : (a))

typedef struct {
	int width;
	int height;
	int format;
	int key_frame;//ÊÇ·ñÎª¹Ø¼üÖ¡
} yuv_params_t;

struct buffer_io_data {
	unsigned char *ptr;
	size_t size; ///< size left in the buffer
};

void *capture_flow_thread(void *s);
AVFormatContext	*video_open_context(char *stream_uri);
int video_get_video_info(AVFormatContext *ff, video_info_t *video,video_pic_capture_context  *context);
int video_malloc_img_convert_buffer(video_info_t *video, video_convert_t *convert);
int video_get_visu_orig_rgb_data(AVFormatContext *ff, video_info_t *video, AVPacket *packet, int min_value,video_pic_capture_context *context);
void *decode_flow_thread(void *s);

#ifdef __cplusplus
}
#endif

#endif
