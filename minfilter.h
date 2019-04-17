/** @file minfilter.h
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

#ifndef _MINFILTER_H_
#define _MINFILTER_H_

#ifdef __cplusplus
extern "C"
{
#endif

void min_filter(const unsigned char *image, unsigned int width,
                unsigned int height, unsigned int ksize,
				unsigned char *minf_image);

#ifdef __cplusplus
}
#endif

#endif