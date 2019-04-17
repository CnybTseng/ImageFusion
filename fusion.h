/** @file fusion.h
 ** @brief Image fusion
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

#ifndef _FUSION_H_
#define _FUSION_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** @typedef struct Fusion
 ** @brief image fusion structure
 **/
struct tagFusion;
typedef struct tagFusion Fusion;

/** @name Create, initialize, and destroy
 ** @{ */
Fusion *fusion_new();
int fusion_init(Fusion *self,
                int base_width, int base_height,
                int unreg_width, int unreg_height);
void fusion_delete(Fusion *self);
/** @} */

/** @name Data operation
 ** @{ */
int fusion_start(Fusion *self);
void fusion_stop(Fusion *self);
int fusion_put(Fusion *self, unsigned char *base, unsigned char *unreg);
int fusion_put_inf(Fusion *self, unsigned char *base);
int fusion_put_vis(Fusion *self, unsigned char *unreg);
int fusion_get(Fusion *self, unsigned char *fu);
int fusion_get_inf(Fusion *self, unsigned char *inf);
int fusion_get_vis(Fusion *self, unsigned char *vis);
int fusion_get_ibf(Fusion *self, unsigned char *ibf);
/** @} */

#ifdef __cplusplus
}
#endif

#endif