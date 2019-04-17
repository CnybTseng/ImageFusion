/** @file registration.h
 ** @brief Image registration
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

#ifndef _REGISTRATION_H_
#define _REGISTRATION_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** @typedef Registration
 ** @brief Image registration
 **/
struct tagRegistration;
typedef struct tagRegistration Registration;

/** @name Create, initialize, and destroy
 ** @{ */
Registration *rm_regist_new();
 
int rm_regist_init(Registration *self,
                   int base_width, int base_height,
				   int unreg_width, int unreg_height,
                   const int *const contrl_points, int npoints,
                   const char *rtf, const char *ctf);

void rm_regist_delete(Registration *self);
/** @} */

/** @name Image registration
 ** @{ */
int rm_regist_warp_image(const Registration *const self,
                         const unsigned char *const src,
						 unsigned char *const dst);
/** @} */
						 
#ifdef __cplusplus
}
#endif

#endif
