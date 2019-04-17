/** @file gaussfilter.h
 ** @brief IIR Gaussian blur filter.
 ** @author Zhiwei Zeng
 ** @date 2018.05.01
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _GAUSSFILTER_H_
#define _GAUSSFILTER_H_

#ifdef __cpluslplus
extern "C"
{
#endif

/** @brief IIR Gaussian filter.
 ** @{ */
void gauss_filter(unsigned char *image,
                  unsigned int width,
				  unsigned int height,
				  float sigma,
				  unsigned char *gf_image);
/** @} */

#ifdef __cpluslplus
}
#endif

#endif