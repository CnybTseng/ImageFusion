/** @file minfilter.c - Implementation
 ** @brief Minimum filter.
 ** @author Zhiwei Zeng
 ** @date 2018.05.04
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
#include <assert.h>

#include "minfilter.h"

/** @brief Minimum filter.
 ** @param image input image.
 ** @param width image width.
 ** @param height image height.
 ** @param minf_image minimum filtered image.
 **/
void min_filter(const unsigned char *image, unsigned int width,
                unsigned int height, unsigned int ksize,
			    unsigned char *minf_image)
{	
	unsigned int krad;
	unsigned int x, y;
	unsigned int kx, ky;
	unsigned int val, minv;
	
	krad = ksize >> 1;
	
	for (y = krad; y < height - krad; y++) {
		for (x = krad; x < width - krad; x++) {
			minv = 0xFF;
			for (ky = y - krad; ky <= y + krad; ky++) {
				for (kx = x - krad; kx <= x + krad; kx++) {
					val = image[ky * width + kx];
					if (val < minv) {
						minv = val;
					}
				}
			}
			minf_image[y * width + x] = minv;
		}
	}
	
	for (y = 0; y < krad; y++) {
		memmove(minf_image + y * width, minf_image + krad * width, width * sizeof(unsigned char));
	}
	
	for (y = height - krad; y < height; y++) {
		memmove(minf_image + y * width, minf_image + (height - krad - 1) * width,
			width * sizeof(unsigned char));
	}
	
	for (y = 0; y < height; y++) {
		for (x = 0; x < krad; x++) {
			minf_image[y * width + x] = minf_image[y * width + krad];
		}
		
		for (x = width - krad; x < width; x++) {
			minf_image[y * width + x] = minf_image[y * width + width - krad - 1];
		}
	}
}