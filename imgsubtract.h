/** @file imgsubtract.h
 ** @brief Image subtraction
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

#ifndef _BKGSUBTRACT_H_
#define _BKGSUBTRACT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** @name Subtract one image from another.
 ** @{ */
void img_subtract_kr(const unsigned char *A, unsigned int width,
                     unsigned int height, const unsigned char *B,
			         unsigned char *C);
void img_subtract(const unsigned char *A, unsigned int width,
                  unsigned int height, const unsigned char *B,
			      short *C);
/** @} */

#ifdef __cplusplus
}
#endif

#endif