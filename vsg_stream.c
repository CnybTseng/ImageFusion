/** @file vsg_stream.c - Implementation
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

#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "vsg_stream.h"
#include "vsg_recorder.h"
#include "vsg_init.h"

int init_video_pic_capture(char *stream)
{
	int flag;
	video_pic_capture_context *context = (video_pic_capture_context *)malloc(sizeof(video_pic_capture_context));
	context->stream=(char *)malloc(strlen(stream));
	memset(context->stream,0,strlen(stream));
	memcpy(context->stream,stream,strlen(stream));
	
	context->ff = video_open_context(stream);
	if(context->ff == NULL) {
		printf("get ffmpeg context error!!!\n");
		return -1;
	}
	
	if( video_get_video_info(context->ff, &context->video,context) == -1) {
		printf("get video info error!!!\n");
		return -1;
	}
	
	//初始化线程属性
	init_thread_attribute(context);
	
	//初始化环形队列
	init_ring_queue(context);
	
	context->runing_flag=1;
	context->pic_index = 0;
	flag=(int)context;
//	printf("flag value is %d\n",flag);
	pthread_mutex_init(&context->out_mutex,NULL);
#ifdef _WIN32
	Sleep(1000);
#else
	sleep(1);
#endif

	return (int)context;
}

int start_video_pic_capture(int handle)
{
	video_pic_capture_context *context = (video_pic_capture_context *)handle;
	int ret;
	pthread_t tid;

	ret = pthread_create(&tid, &context->pattr, decode_flow_thread, context);
	printf("*********ret is %d",ret);
	if (ret != 0) {
		perror("create decode_flow_thread");
		return -1;
	}
	
	//取相机视频流线程
	ret = pthread_create(&tid,&context->pattr, capture_flow_thread, context);
	if (ret != 0) {
		perror("create capture_flow_thread");
		return -1;
	}
	
	return 0;
}

void pause_video_pic_capture(int handle )
{
	video_pic_capture_context *context = (video_pic_capture_context *)handle;
	avformat_close_input(&context->ff);
}

int resume_video_pic_capture(int handle,char *stream )
{
	video_pic_capture_context *context = (video_pic_capture_context *)handle;
	context->ff = video_open_context(stream);
	if(context->ff == NULL) {
		printf("get ffmpeg context error!!!\n");
		return -1;
	}
	if( video_get_video_info(context->ff, &context->video,context) == -1) {
		printf("get video info error!!!\n");
		return -1;
	}
//	av_read_play(context->ff);

	return 0;
}

void stop_video_pic_capture(int handle )
{
	video_pic_capture_context *context = (video_pic_capture_context *)handle;
	context->runing_flag=0;
#ifdef _WIN32
	Sleep(1000);
#else
	sleep(1);
#endif
	avcodec_close(context->video.codec);
	avformat_close_input(&context->ff);		// 已包含avformat_free_context(pFormatCtx)操作;
}

void free_video_pic_capture(int handle)
{
	video_pic_capture_context *context = (video_pic_capture_context *)handle;
	// free convert
	av_frame_free(&context->picture_convert.yuv_frame);
	av_frame_free(&context->picture_convert.rgb_frame);
	// free(context->picture_convert.out_buf);
	av_free(context->picture_convert.out_buf);
	// video_free_img_convert_buffer(&context->video, &context->picture_convert);
	sws_freeContext(context->picture_convert.img_convert_ctx);
	
	int i;
	for(i=0; i<RING_BUF_NUM; i++) {
		av_packet_free(&context->ring_zone->picture[i].packet);
	}
}

video_pic_reader *create_video_pic_reader(int capture_handle)
{
	video_pic_reader *ret = (video_pic_reader *)malloc(sizeof(video_pic_reader));
	ret->last_pic_index = 0;
	ret->capture_handle = (video_pic_capture_context *)capture_handle;
	return ret;
}

rgb_pic *capture_video_rgb_data(video_pic_reader *reader)
{

	video_pic_capture_context *context = (video_pic_capture_context *)reader->capture_handle;
	
	rgb_pic *rgb_pack = (rgb_pic *)malloc(sizeof(rgb_pic));
	rgb_pack->data=(unsigned char *)malloc(sizeof(unsigned char)*context->video.codec->height*context->video.codec->width*3);
	
	while(reader->last_pic_index==context->pic_index) {
#ifdef _WIN32
		Sleep(10);
#else
		usleep(10000);
#endif
	}
	
	reader->last_pic_index=context->pic_index;
	pthread_mutex_lock(&context->out_mutex);
	memcpy(context->picture_convert.yuv_frame,context->save_frame,sizeof(AVFrame));
	pthread_mutex_unlock(&context->out_mutex);
	
	sws_scale(context->picture_convert.img_convert_ctx, (const uint8_t* const*)context->picture_convert.yuv_frame->data,\
	          context->picture_convert.yuv_frame->linesize, 0, context->video.codec->height,context->picture_convert.rgb_frame->data,\
	          context->picture_convert.rgb_frame->linesize);
	
	memcpy(rgb_pack->data,context->picture_convert.rgb_frame->data[0],sizeof(unsigned char)*context->video.codec->height*context->video.codec->width*3);
	rgb_pack->height=context->video.codec->height;
	rgb_pack->width=context->video.codec->width;
	
	return rgb_pack;
}

void free_video_rgb_pic(rgb_pic *pic)
{
	free(pic->data);
	free(pic);
}

void free_video_pic_reader(video_pic_reader *reader)
{
	free(reader);
}

yuv_pic *capture_video_yuv_data(video_pic_reader *reader)
{
	video_pic_capture_context *context = (video_pic_capture_context *)reader->capture_handle;
	
	yuv_pic *yuv_pack = (yuv_pic *)malloc(sizeof(yuv_pic));
	yuv_pack->data=(unsigned char *)malloc(sizeof(unsigned char)*context->video.codec->height*context->video.codec->width*3>>1);
	memset(yuv_pack->data, 0,context->video.codec->height * context->video.codec->width * 3 >>1);
	int a = 0, i;
	
	while(reader->last_pic_index==context->pic_index) {
#ifdef _WIN32
		Sleep(3);
#else
		usleep(10000);
#endif
	}
	
	reader->last_pic_index=context->pic_index;
	pthread_mutex_lock(&context->out_mutex);
	memcpy(context->picture_convert.yuv_frame,context->save_frame,sizeof(AVFrame));
	
	for (i = 0; i<context->video.codec->height; i++) {
		memcpy(yuv_pack->data + a, context->save_frame->data[0] + i * context->save_frame->linesize[0], context->video.codec->width);
		a += context->video.codec->width;
	}
	
	for (i = 0; i<context->video.codec->height/2; i++) {
		memcpy(yuv_pack->data + a, context->save_frame->data[1] + i * context->save_frame->linesize[1], context->video.codec->width>>1);
		a += context->video.codec->width/2;
	}
	
	for (i = 0; i<context->video.codec->height/2; i++) {
		memcpy(yuv_pack->data + a, context->save_frame->data[2] + i * context->save_frame->linesize[2], context->video.codec->width>>1);
		a += context->video.codec->width/2;
	}
	
	pthread_mutex_unlock(&context->out_mutex);
	yuv_pack->height=context->video.codec->height;
	yuv_pack->width=context->video.codec->width;
	yuv_pack->codec=context->video.codec;
	
	return yuv_pack;
}

void free_video_yuv_pic(yuv_pic *pic)
{
	free(pic->data);
	free(pic);
}

rgb_pic *yuv_to_rgb_data(video_pic_reader *reader)
{

	video_pic_capture_context *context = (video_pic_capture_context *)reader->capture_handle;
	rgb_pic *rgb_pack = (rgb_pic *)malloc(sizeof(rgb_pic));
	rgb_pack->data=(unsigned char *)malloc(sizeof(unsigned char)*context->video.codec->height*context->video.codec->width*3);
	sws_scale(context->picture_convert.img_convert_ctx, (const uint8_t* const*)context->picture_convert.yuv_frame->data,\
	          context->picture_convert.yuv_frame->linesize, 0, context->video.codec->height,context->picture_convert.rgb_frame->data,\
	          context->picture_convert.rgb_frame->linesize);
	memcpy(rgb_pack->data,context->picture_convert.rgb_frame->data[0],sizeof(unsigned char)*context->video.codec->height*context->video.codec->width*3);
	rgb_pack->height=context->video.codec->height;
	rgb_pack->width=context->video.codec->width;
	return rgb_pack;
}

rgb_pic *yuv_to_rgb(yuv_pic *pic,int width,int height)
{

	uint8_t *inbuf[4];
	uint8_t *outbuf[4];
	struct SwsContext *img_convert_ctx;
	img_convert_ctx = sws_getContext(pic->width, pic->height,pic->codec->pix_fmt,\
	                                 width, height, AV_PIX_FMT_RGB24,SWS_BICUBIC,NULL, NULL, NULL);
	int ret=sws_setColorspaceDetails(img_convert_ctx,sws_getCoefficients(SWS_CS_ITU601),1,\
	                                 sws_getCoefficients(SWS_CS_ITU709),1,\
	                                 0, 1 << 16, 1 << 16);
	if (ret==-1)
		printf( "Colorspace not support.\n");
	
	int inlinesize[4] = {pic->width, pic->width/2, pic->width/2, 0};
	int outlinesize[4]= {width*3, width*3, width*3, 0};

	inbuf[0] = (uint8_t *)malloc(sizeof(unsigned char)*pic->width*pic->height);
	inbuf[1] = (uint8_t *)malloc(sizeof(unsigned char)*pic->width*pic->height>>2);
	inbuf[2] = (uint8_t *)malloc(sizeof(unsigned char)*pic->width*pic->height>>2);
	inbuf[3] = NULL;

	outbuf[0] = (uint8_t *)malloc(sizeof(unsigned char)*width*height*3);
	outbuf[1] = NULL;
	outbuf[2] = NULL;
	outbuf[3] = NULL;

	rgb_pic *rgb_pack = (rgb_pic *)malloc(sizeof(rgb_pic));
	rgb_pack->data=(unsigned char *)malloc(sizeof(unsigned char)*height*width*3);

	memcpy(inbuf[0], pic->data,pic->width*pic->height);
	memcpy(inbuf[1], pic->data+pic->width*pic->height, pic->width*pic->height>>2);
	memcpy(inbuf[2], pic->data+(pic->width*pic->height*5>>2), pic->width*pic->height>>2);

	sws_scale(img_convert_ctx,inbuf,inlinesize, 0,pic->height,outbuf,outlinesize);
	memcpy(rgb_pack->data, outbuf[0],width*height*3);
	rgb_pack->height=height;
	rgb_pack->width=width;
	free(inbuf[0]);
	free(inbuf[1]);
	free(inbuf[2]);
	free(outbuf[0]);
	sws_freeContext(img_convert_ctx);
	
	return rgb_pack;
}