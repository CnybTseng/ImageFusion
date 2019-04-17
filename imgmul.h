/** @file imgmul.h
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

#ifndef _IMGMUL_H_
#define _IMGMUL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** @name A image multiply a scalar.
 ** @{ */
void img_mul_s_kr(const unsigned char *A, unsigned int width,
                  unsigned int height, float k, unsigned char *B);
/** @} */

#ifdef __cplusplus
}
#endif

#endif