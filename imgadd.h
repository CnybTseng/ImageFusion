/** @file imgadd.h
 ** @brief Image add
 ** @author Zhiwei Zeng
 ** @date 2018.04.25
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _IMGADD_H_
#define _IMGADD_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** @name Add two images.
 ** @{ */
void img_add_kr(const unsigned char *A, unsigned int width,
                unsigned int height, const unsigned char *B,
			    unsigned char *C);
void img_add(const unsigned char *A, unsigned int width,
             unsigned int height, const unsigned char *B,
			 unsigned short *C);
/** @} */

#ifdef __cplusplus
}
#endif

#endif