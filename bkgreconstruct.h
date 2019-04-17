/** @file bkgreconstruct.h
 ** @brief Infrared background reconstruction with quadtree decompose and Bezier interpolation
 ** @author Zhiwei Zeng
 ** @date 2018.04.20
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _BKGRECONSTRUCT_H_
#define _BKGRECONSTRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** @typedef struct BkgReconst
 ** @brief background reconstruction structure
 **/
struct tagBkgReconst;
typedef struct tagBkgReconst BkgReconst;

/** @name Create, initialize, and destroy
 ** @{ */
BkgReconst *bkgreconst_new();
int bkgreconst_init(BkgReconst *self, unsigned int width,
                    unsigned int height);
void bkgreconst_delete(BkgReconst *self);
/** @} */

/** @name Data processing
 ** @{ */
int bkgreconst_start(BkgReconst *self);
void bkgreconst_stop(BkgReconst *self);
int bkgreconst_put(BkgReconst *self,
                   unsigned char *image);
int bkgreconst_get(BkgReconst *self,
                   unsigned char *bkg);
/** @} */

#ifdef __cplusplus
}
#endif

#endif