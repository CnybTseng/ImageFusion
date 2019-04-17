/** @file quadtree.h
 ** @brief Quadtree
 ** @author Zhiwei Zeng
 ** @date 2018.04.19
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _QUADTREE_H_
#define _QUADTREE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** @typedef struct Quadrant
 ** @brief quadrant structure
 **/
typedef struct
{
	unsigned int top;		/**< y position of top side. */
	unsigned int left;		/**< x position of left side. */
	unsigned int bottom;	/**< y position of bottom side. */
	unsigned int right;		/**< x position of right side. */
}Quadrant;

/** @typedef struct Blob
 ** @brief blob structure
 **/
typedef struct
{
	Quadrant quad;			/**< blob position. */
	unsigned int range;		/**< blob gray range. */
}Blob;

/** @typedef struct QTree
 ** @brief quadtree structure
 **/
struct tagQTree;
typedef struct tagQTree QTree;

/** @name Create, Initialize, and destroy
 ** @{ */
QTree *qtree_new();
void qtree_init(QTree *self, unsigned int minbw,
                unsigned int minbh, unsigned int mingr);
void qtree_delete(QTree *self);
/** @} */

/** @name decompose image with quadtree
 ** @{ */
void qtree_decompose(QTree *self,
                     const unsigned char *image,
					 unsigned int width,
					 unsigned int height);
int gtree_get_leafnode(QTree *self, Blob *blobs);
void qtree_reset(QTree *self);
/** @} */

#ifdef __cplusplus
}
#endif

#endif