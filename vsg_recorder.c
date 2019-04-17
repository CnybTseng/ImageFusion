/** @file vsg_recorder.c - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "vsg_recorder.h"
#include "vsg_init.h"
#include "vsg_ring.h"

void *capture_flow_thread(void *s)
{

	video_pic_capture_context *context = (video_pic_capture_context *)s;

	printf("**********malloc_img_convert_buffer is ok*************\n");
	while(context->runing_flag) {
		video_get_visu_orig_rgb_data(context->ff,&context->video,context->ring_zone->picture[context->ring_zone->in].packet, 100,context);
	}
	return (void *)(-1);
}

/*打开取可见光相机原始数据环境*/
AVFormatContext	*video_open_context(char *stream_uri)
{
	AVFormatContext	*rtsp_fmt_ctx;//Format I/O context
//	init_queue(&recvqueue, 1024*1024*10);
	//注册编解码器
	/*注册所有组件*/
	av_register_all();
	/*网络支持*/
	avformat_network_init();
	/*申请AVFormatContext实体*/
	rtsp_fmt_ctx = avformat_alloc_context();//初始化AVFormatContext
	if(rtsp_fmt_ctx == NULL) {
		printf("alloc avformat fail\n");
		return NULL;
	}

	/* 使用tcp连接 */
	AVDictionary *option = NULL;
	av_dict_set(&option, "rtsp_transport", "tcp", 0);
	av_dict_set(&option, "stimeout", "6000000", 0); //如果没有设置stimeout，那么把ipc网线拔掉，av_read_frame会阻塞（时间单位是微妙）
	if(avformat_open_input(&rtsp_fmt_ctx, stream_uri,NULL,&option)!=0) { //得到AVStream信息
		printf("Couldn't open input stream.\n");
		avformat_close_input(&rtsp_fmt_ctx);
		return NULL;
	}
	return rtsp_fmt_ctx;
}

/*打开取可见光相机原始数据环境*/
int video_get_video_info(AVFormatContext *ff, video_info_t *video,video_pic_capture_context *context)
{
	unsigned int  i;
	context->pCodecParserCtx = NULL;
	/*得到流信息*/
	if(avformat_find_stream_info(ff, NULL)<0) { //得到更多AVStream信息
		printf("Couldn't find stream information.\n");
		return -1;
	}

	for(i=0; i< ff->nb_streams; i++) {
		/*视频流*/
		if(ff->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
			video->index = i;
			printf("video stream id is %d\n", video->index);
			break;
		}
	}
	/*通过编码器id找开解码器*/
	video->codec= ff->streams[video->index]->codec;
	context->rtsp_codec = avcodec_find_decoder(video->codec->codec_id);
	if(context->rtsp_codec == NULL) {
		printf("Codec not found.\n");
		return -1;
	}
	if(context->rtsp_codec->capabilities&AV_CODEC_CAP_TRUNCATED) {
		video->codec->flags |= AV_CODEC_FLAG_TRUNCATED; /* we dont send complete frames */
		//保证视频流是一帧一帧的送到解码器的
	}
	context->rtsp_codec->capabilities |= AV_CODEC_CAP_DELAY;
	video->codec->active_thread_type |= AV_CODEC_CAP_FRAME_THREADS;
	video->codec->active_thread_type|=FF_THREAD_FRAME;
	/*打开解码器*/
	if(avcodec_open2(video->codec,context->rtsp_codec, NULL)<0) {
		printf("Could not open codec.\n");
		return -1;
	}
	context->pCodecParserCtx = av_parser_init(video->codec->codec_id);
	context->pCodecParserCtx->flags |= PARSER_FLAG_ONCE;
	return 0;
}

/*打开取可见光相机原始数据环境*/
int video_malloc_img_convert_buffer(video_info_t *video, video_convert_t *convert)
{
	// YUV数据存储空间
//    int yuv420p_bytes = avpicture_get_size(AV_PIX_FMT_YUV420P,video->codec->width,video->codec->height);
	convert->yuv_frame = av_frame_alloc();
	if(convert->yuv_frame == NULL) {
		printf("yuv frame malloc fail!!!\n");
		return -1;
	}

	// RGB数据存储空间
	convert->rgb_frame = av_frame_alloc();
	if(convert->rgb_frame == NULL) {
		printf("rgb frame malloc fail!!!\n");
		av_frame_free(&(convert->yuv_frame));
		return -1;
	}
	convert->out_buf =  (uint8_t *)av_malloc(avpicture_get_size(AV_PIX_FMT_RGB24,video->codec->width,video->codec->height)*sizeof(uint8_t));
	if(convert->out_buf == NULL) {
		printf("out buffer malloc fail!!!\n");
		av_frame_free(&(convert->yuv_frame));
		av_frame_free(&(convert->rgb_frame));
		return -1;
	}

	avpicture_fill((AVPicture *)(convert->rgb_frame), convert->out_buf,AV_PIX_FMT_RGB24, video->codec->width,  video->codec->height);
	convert->img_convert_ctx = sws_getContext(video->codec->width, video->codec->height, video->codec->pix_fmt, \
	                           video->codec->width, video->codec->height, AV_PIX_FMT_RGB24, SWS_BICUBIC, NULL, NULL, NULL);
	if(convert->img_convert_ctx == NULL) {
		printf("img_convert_ctx get error!!!\n");
		av_frame_free(&(convert->yuv_frame));
		av_frame_free(&(convert->rgb_frame));
		free(convert->out_buf);
		return -1;
	}
	return 0;
}

int video_get_visu_orig_rgb_data(AVFormatContext *ff, video_info_t *video, AVPacket *packet, int min_value,video_pic_capture_context *context)
{
	int ret,handle;
	handle=(int)context;
//	printf("******packet size afer free is %d********\n",sizeof(packet));
	if((ret=av_read_frame(ff,packet))< 0) {
		printf("****can't read data from stream %s*****\n",context->stream);
		while(resume_video_pic_capture(handle,context->stream)==-1) {
#ifdef _WIN32
			Sleep(10);
#else
			usleep(10000);;
#endif
		}
		return -1;
	}
//		printf("******packet size afer read data is %d********\n",sizeof(packet));
	if(packet->stream_index != video->index) {
		return -1;
	}
	if(packet->size==0) {
		return -1;
	}
	ring_put_picture_packet(context->ring_zone);
	return 0;
}

void *decode_flow_thread(void *s)
{
//    char vs_alarm_frm_path[128] = {0};
	int img_num=0,finished;
	int index,ret,pict_type_flag=0;
	video_pic_capture_context *context = (video_pic_capture_context *)s;
	int begin_flag=0;
	while(context->runing_flag) {

		if(ring_get_picture_packet(context->ring_zone)==-1) {
#ifdef _WIN32
			Sleep(10);
#else
			usleep(10000);
#endif
			continue;
		}
		if(context->ring_zone->out==0&&begin_flag!=0) {
			index=RING_BUF_NUM-1;
			pict_type_flag=0;
		} else {
			index=context->ring_zone->out-1;
			begin_flag=1;
		}
		ret = avcodec_decode_video2(context->video.codec,context->yuv_frame, &finished,context->ring_zone->picture[index].packet);
		av_free_packet(context->ring_zone->picture[index].packet);
		if(ret <0) {
			printf("\n********can't decode picture data ********************\n");
			continue;
		}
		/*下面的判断语句需后面进一步优化*/
		if(index<=I_FRAME_INTERVAL&&context->yuv_frame->pict_type==AV_PICTURE_TYPE_I) {
			pict_type_flag=1;
		}
		if(1!=pict_type_flag) {
			continue;
		}
		if(finished) {
			pthread_mutex_lock(&context->out_mutex);
			memcpy(context->save_frame,context->yuv_frame,sizeof(AVFrame));
			context->pic_index++;
			pthread_mutex_unlock(&context->out_mutex);
		} else {
			printf("********can't decode picture data ********************\n");
			continue;
		}
		img_num++;
	}
	return (void *)(-1);
}
