/** @file main.c
 ** @brief Image fusion testing
 ** @author Zhiwei Zeng
 ** @date 2018.04.16
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
#include <conio.h>
#include <windows.h>

#include "fifo.h"
#include "pthread.h"
#include "fusion.h"
#include "raw_frame.h"
#include "vsg_stream.h"

#ifndef SDL_MAIN_HANDLED
#	define SDL_MAIN_HANDLED
#	include "SDL.h"
#endif

static unsigned int roundup_power_of_2(unsigned int a);
static int capture_infrared_image_start();
static void *capture_infrared_image(void *s);
static int capture_visual_image_start();
static void *capture_visual_image_thread(void *);
static void process_image_boder(unsigned short *image, unsigned int width,
                                unsigned int height);

static int sdl_quited = 0;	
static Fusion *fusion = NULL;					 
static Fifo *inf_ring = NULL;
static Fifo *vis_ring = NULL;
static unsigned int base_width = 384;
static unsigned int base_height = 288;
static unsigned int ureg_width = 1920;
static unsigned int ureg_height = 1080;
static unsigned int base_image_size;
static unsigned int ureg_image_size;
						 
int main(int argc, char *argv[])
{
	int read_len = 0;
	const int caches = 4;
	unsigned char *base_image = NULL;
	unsigned char *ureg_image = NULL;
	unsigned char *fusn_image = NULL;
	unsigned char *gsci_image = NULL;
	unsigned char *regt_image = NULL;
	unsigned char *ibrf_image = NULL;
	FILE *fp = NULL;
	SDL_Surface *ico = NULL;
	SDL_Window *window = NULL;
	SDL_Renderer *renderer = NULL;
	SDL_Texture *texture = NULL;
	SDL_Rect rect1;
	SDL_Rect rect2;
	SDL_Rect rect3;
	SDL_Rect rect4;
	SDL_Event event;
	int mx, my;
	int x1, y1;
	int x2, y2;
	
	LARGE_INTEGER frequency;
	LARGE_INTEGER start_count = {0, 0};
	LARGE_INTEGER stop_count = {0, 0}; 
	double run_time = 0;
	
	QueryPerformanceFrequency(&frequency); 
	
	base_image_size = roundup_power_of_2(base_width * base_height * sizeof(unsigned short));
	ureg_image_size = roundup_power_of_2(ureg_width * ureg_height * 3 >> 1);
	
	inf_ring = fifo_alloc(caches * base_image_size);
	if (!inf_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	vis_ring = fifo_alloc(caches * ureg_image_size);
	if (!vis_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	base_image = (unsigned char *)malloc(base_image_size);
	if (!base_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	ureg_image = (unsigned char *)malloc(ureg_image_size);
	if (!ureg_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	fusn_image = (unsigned char *)malloc(base_width * base_height * sizeof(unsigned char) * 3 >> 1);
	if (!fusn_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	gsci_image = (unsigned char *)malloc(base_width * base_height * sizeof(unsigned char) * 3 >> 1);
	if (!gsci_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	regt_image = (unsigned char *)malloc(base_width * base_height * sizeof(unsigned char) * 3 >> 1);
	if (!regt_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	ibrf_image = (unsigned char *)malloc(base_width * base_height * sizeof(unsigned char) * 3 >> 1);
	if (!ibrf_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	fusion = fusion_new();
	if (!fusion) {
		fprintf(stderr, "fusion_new fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	if (fusion_init(fusion, base_width, base_height, ureg_width, ureg_height)) {
		fprintf(stderr, "fusion_init fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	if (fusion_start(fusion)) {
		fprintf(stderr, "fusion_start fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	if (capture_infrared_image_start()) {
		fprintf(stderr, "capture_infrared_image_start fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	if (capture_visual_image_start()) {
		fprintf(stderr, "capture_visual_image_start fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
		
	if (SDL_Init(SDL_INIT_VIDEO)) {
		fprintf(stderr, "SDL_Init fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	ico = SDL_LoadBMP("zlt.bmp");
	if (!ico) {
		fprintf(stderr, "SDL_LoadBMP fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	window = SDL_CreateWindow("Image Fusion Demo",
                              SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED,
                              base_width * 2, base_height * 2,
                              SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL);
	if (!window) {
		fprintf(stderr, "SDL_CreateWindow fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
		
	SDL_SetWindowIcon(window, ico);
	
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	if (!renderer) {
		fprintf(stderr, "SDL_CreateRenderer fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING,
		base_width * 2, base_height * 2); 
	if (!texture) {
		fprintf(stderr, "SDL_CreateTexture fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	rect1 = {0, 0, base_width, base_height};
	rect2 = {base_width, 0, base_width, base_height};
	rect3 = {0, base_height, base_width, base_height};
	rect4 = {base_width, base_height, base_width, base_height};
		
	memset(ibrf_image + base_width * base_height, 0x80, base_width * base_height >> 1);
	
	while (!sdl_quited) {		
		/*read_len = fifo_get(inf_ring, (char *)base_image, base_image_size);
		if (read_len != base_image_size) {
			continue;
		}
		
		while (!sdl_quited) {
			read_len = fifo_get(vis_ring, (char *)ureg_image, ureg_image_size);
			if (read_len == ureg_image_size) {
				break;
			}
		}
		
		if (fusion_put(fusion, (unsigned char *)base_image, ureg_image)) {
			goto SDL_handle;
		}*/

		if (fusion_get(fusion, fusn_image) && fusion_get_inf(fusion, gsci_image) &&
			fusion_get_vis(fusion, regt_image) && fusion_get_ibf(fusion, ibrf_image)) {			
			if (sdl_quited) {
				break;
			}
						
			SDL_UpdateTexture(texture, &rect1, gsci_image, base_width);  
			SDL_UpdateTexture(texture, &rect2, regt_image, base_width); 
			SDL_UpdateTexture(texture, &rect3, ibrf_image, base_width); 
			SDL_UpdateTexture(texture, &rect4, fusn_image, base_width); 
            SDL_RenderClear(renderer);  
			SDL_RenderCopy(renderer, texture, NULL, NULL);
			SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
			SDL_RenderDrawLine(renderer, 0, y1, (base_width << 1) - 1, y1);
			SDL_RenderDrawLine(renderer, 0, y2, (base_width << 1) - 1, y2);
			SDL_RenderDrawLine(renderer, x1, 0, x1, (base_height << 1) - 1);
			SDL_RenderDrawLine(renderer, x2, 0, x2, (base_height << 1) - 1);
            SDL_RenderPresent(renderer);
		} 
		
		// SDL_handle:
		SDL_Delay(3);
		SDL_PollEvent(&event);  
        switch (event.type) {  
        case SDL_QUIT:  
			sdl_quited = 1;
            break;
		case SDL_MOUSEMOTION:
			SDL_GetMouseState(&mx, &my);
			x1 = mx;
			y1 = my;
			x2 = (mx + base_width) % (base_width << 1);
			y2 = (my + base_height) % (base_height << 1);
			break;
        default:  
            break;  
        }
	}
	
	fusion_stop(fusion);
	
	fp = fopen("gsci.dat", "wb");
	if (!fp) {
		fprintf(stderr, "fopen fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	fwrite(gsci_image, sizeof(unsigned char), base_width * base_height * sizeof(unsigned char), fp);
	fclose(fp);
	
	fp = fopen("fusn.dat", "wb");
	if (!fp) {
		fprintf(stderr, "fopen fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	fwrite(fusn_image, sizeof(unsigned char), base_width * base_height * sizeof(unsigned char) * 3 >> 1, fp);
	fclose(fp);
	
	clean:
	Sleep(1000);
	
	if (inf_ring) {
		fifo_delete(inf_ring);
	}
	
	if (vis_ring) {
		fifo_delete(vis_ring);
	}
	
	if (base_image) {
		free(base_image);
		base_image = NULL;
	}

	if (ureg_image) {
		free(ureg_image);
		ureg_image = NULL;
	}
	
	if (fusn_image) {
		free(fusn_image);
		fusn_image = NULL;
	}
	
	if (gsci_image) {
		free(gsci_image);
		gsci_image = NULL;
	}
	
	if (regt_image) {
		free(regt_image);
		regt_image = NULL;
	}
	
	if (ibrf_image) {
		free(ibrf_image);
		ibrf_image = NULL;
	}
	
	if (fusion) {
		fusion_delete(fusion);
	}
	
	if (fp) {
		fclose(fp);
	}
		
	if (texture) {
		SDL_DestroyTexture(texture); 
	}
	
	if (renderer) {
		SDL_DestroyRenderer(renderer);
	}
	
	if (ico) {
		SDL_FreeSurface(ico);
	}
	
	if (window) {
		SDL_DestroyWindow(window);
	}
	
	SDL_Quit();
	
	return 0;
}

unsigned int roundup_power_of_2(unsigned int a)
{
	unsigned int position;
	int i;
	
	if (a == 0) {
		return 0;
	}

	position = 0;
	for (i = a; i != 0; i >>= 1) {
		position++;
	}

	return (unsigned int)(1 << position);
}

int capture_infrared_image_start()
{
	pthread_t tid;
	pthread_attr_t attr;
	int ret;
	
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	ret = pthread_create(&tid, &attr, capture_infrared_image, NULL);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	
	return 0;
}

void *capture_infrared_image(void *s)
{
	void *ihandle = NULL;
	int iw, ih;
	unsigned short *image = NULL;
	int len = 0;
	
	image = (unsigned short *)malloc(base_image_size);
	if (!image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	ihandle = frame_receiver_new();
	if (!ihandle) {
		fprintf(stderr, "frame_receiver_new fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	if (!frame_receiver_init(ihandle, 32345)) {
		fprintf(stderr, "frame_receiver_init fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	while (!sdl_quited) {
		if (!frame_receiver_get(ihandle, image, &iw, &ih)) {
			continue;
		}
		
		process_image_boder(image, base_width, base_height);
		/*len = fifo_put(inf_ring, (char *)image, base_image_size);
		if (len != base_image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}*/
		
		fusion_put_inf(fusion, (unsigned char *)image);
	}
	
	clean:
	if (image) {
		free(image);
		image = NULL;
	}
	
	if (ihandle) {
		frame_receiver_stop(ihandle);
		frame_receiver_free(ihandle);
	}
	
	return (void *)(0);
}

int capture_visual_image_start()
{
	pthread_t tid;
	pthread_attr_t attr;
	int ret;
	
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	ret = pthread_create(&tid, &attr, capture_visual_image_thread, NULL);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	
	return 0;
}

void *capture_visual_image_thread(void *)
{
	char *url = NULL;
	int vhandle = -1;
	video_pic_reader *reader = NULL;
	yuv_pic *yuv_pack = NULL;
	char *image = NULL;
	int len = 0;
	
	url = "rtsp://admin:zlt123456@192.168.9.64:554/Streaming/Channels/101?transportmode=unicast&profile=Profile_1";
	
	image = (char *)malloc(ureg_image_size);
	if (!image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	vhandle = init_video_pic_capture(url);
	if (-1 == vhandle) {
		fprintf(stderr, "init_video_pic_capture fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}

	if (start_video_pic_capture(vhandle)) {
		fprintf(stderr, "start_video_pic_capture fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}

	reader = create_video_pic_reader(vhandle);
	if (!reader) {
		fprintf(stderr, "create_video_pic_reader fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	while (!sdl_quited) {
		yuv_pack = capture_video_yuv_data(reader);
		if (!yuv_pack) {
			fprintf(stderr, "capture_video_yuv_data fail[%s:%d].\n", __FILE__, __LINE__);
			break;
		}
		
		/*memmove(image, yuv_pack->data, ureg_width * ureg_height * 3 >> 1);
		free_video_yuv_pic(yuv_pack);
		
		len = fifo_put(vis_ring, image, ureg_image_size);
		if (len != ureg_image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}*/
		
		fusion_put_vis(fusion, yuv_pack->data);
		free_video_yuv_pic(yuv_pack);
	}
	
	clean:
	if (image) {
		free(image);
		image = NULL;
	}
	
	if (-1 != vhandle) {
		if (reader) {
			free_video_pic_reader(reader);
		}
		
		stop_video_pic_capture(vhandle);
		free_video_pic_capture(vhandle);
	}
	
	return (void *)(0);
}

void process_image_boder(unsigned short *image, unsigned int width,
                         unsigned int height)
{
	unsigned int y;
	unsigned short *iptr = NULL;
	
	memmove(image, image + width, width * sizeof(unsigned short));
	iptr = image;
	
	for (y = 0; y < height; y++) {
		*(iptr) = *(iptr + 2);
		*(iptr + 1) = *(iptr + 2);
		iptr += width;
	}
}