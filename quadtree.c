/** @file quadtree.c - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "quadtree.h"

/** @typedef enum Corner
 ** @brief enumerate corner
 **/
typedef enum
{
	TLC = 1,						/**< top left corner. */
	TRC,							/**< top right corner. */
	LLC,							/**< lower left corner. */
	LRC								/**< lower right corner. */
}Corner;

/** @typedef struct QNode
 ** @brief quadtree node structure
 **/
typedef struct tagQNode
{
	Blob blob;						/**< image blob. */
	struct tagQNode *next[4];		/**< children nodes. */
}QNode;

struct tagQTree
{
	QNode *root;					/**< root node. */
	unsigned int depth;				/**< quadtree depth. */
	unsigned int minbw;				/**< minimum blob width. */		
	unsigned int minbh;				/**< minimum blob height. */
	unsigned int mingr;				/**< minimum gray range. */
};

/** @name some private functions
 ** @{ */
static void split_blob(QTree *self, QNode *root, Quadrant quad,
                       const unsigned char *image, unsigned int width,
					   unsigned int height);
static void get_leafnode(QNode *root, Blob *blobs, int *nblobs);
static QNode *add_node(QTree *self, Blob *blob);
static void add_child_node(QNode *root, QNode *node);
static int which_child_of_father(const Blob *father, const Blob *child);
static void delete_all_nodes(QNode *root);
/** @} */

/** @brief Create a new instance of quadtree.
 ** @return a new instance of quadtree.
 **/
QTree *qtree_new()
{
	QTree *self = (QTree *)malloc(sizeof(QTree));
	return self;
}

/** @brief Initialize quadtree.
 ** @param self quadtree instance.
 ** @param minbw minimum blob width.
 ** @param minbh minimum blob height.
 ** @param mingr minimum blob gray range.
 **/
void qtree_init(QTree *self, unsigned int minbw,
                unsigned int minbh, unsigned int mingr)
{
	assert(self);
	
	self->root = NULL;
	self->depth = 0;
	self->minbw = minbw;
	self->minbh = minbh;
	self->mingr = mingr;
}

/** @brief Delete quadtree instance.
 ** @param self quadtree instance.
 **/
void qtree_delete(QTree *self)
{
	if (self) {
		delete_all_nodes(self->root);
		
		self->root = NULL;
		self->depth = 0;
		
		free(self);
		self = NULL;
	}
}

/** @brief Decompose image into blobs with quadtree structure.
 ** @param self quadtree instance.
 ** @param image single channel image.
 ** @param width image width.
 ** @param height image height.
 **/
void qtree_decompose(QTree *self,
                     const unsigned char *image,
					 unsigned int width,
					 unsigned int height)
{
	QNode *root = NULL;
	Quadrant quad;
	
	assert(self);
	assert(image);
	
	quad.top = 0;
	quad.left = 0;
	quad.bottom = height;
	quad.right = width;
	
	split_blob(self, root, quad, image, width, height);
}

/** @brief Get leaf node of quadtree.
 ** @param self quadtree instance.
 ** @param blobs allocated blob buffer.
 ** @return the number of leaf nodes.
 **/
int gtree_get_leafnode(QTree *self, Blob *blobs)
{
	int nblobs = 0;
	
	assert(self);
	assert(blobs);

	get_leafnode(self->root, blobs, &nblobs);
	
	return nblobs;
}

/** @brief Reset quadtree.
 ** @param self quadtree instance.
 **/
void qtree_reset(QTree *self)
{
	assert(self);
	
	delete_all_nodes(self->root);
	
	self->root = NULL;
	self->depth = 0;
}

/** @brief Split image blob with quadtree.
 ** @param self quadtree instance.
 ** @param root root node.
 ** @param quad blob quadrant.
 ** @param image single channel image.
 ** @param width image width.
 ** @param height image height.
 **/
void split_blob(QTree *self, QNode *root, Quadrant quad,
                const unsigned char *image, unsigned int width,
				unsigned int height)
{
	Blob blob;
	unsigned int range;
	unsigned char *rptr;
	unsigned char val;
	unsigned char minval = 0xFF;
	unsigned char maxval = 0;
	unsigned int bw, bh;
	unsigned int x, y;
	unsigned int horizon_middle;
	unsigned int vertical_middle;
	Quadrant tlc_blob;
	Quadrant trc_blob;
	Quadrant llc_blob;
	Quadrant lrc_blob;

	/* blob gray range. */
	for (y = quad.top; y < quad.bottom; y++) {
		rptr = (unsigned char *)(image + y * width);
		for (x = quad.left; x < quad.right; x++) {
			val = *(rptr + x);
			if (val < minval) {
				minval = val;
			}
			if (val > maxval) {
				maxval = val;
			}
		}
	}
	
	bw = quad.right - quad.left;
	bh = quad.bottom - quad.top;
	range = maxval - minval;

	blob.quad.top = quad.top;
	blob.quad.left = quad.left;
	blob.quad.bottom = quad.bottom;
	blob.quad.right = quad.right;
	blob.range = range;

	/* add node to tree. */
	root = add_node(self, &blob);
	
	/* split blob if possible. */
	if (bw > self->minbw && bh > self->minbh && range > self->mingr) {
		horizon_middle = (quad.top + quad.bottom) >> 1;
		vertical_middle = (quad.left + quad.right) >> 1;
		
		/* top left corner. */
		tlc_blob.top = quad.top;
		tlc_blob.left = quad.left;
		tlc_blob.bottom = horizon_middle;
		tlc_blob.right = vertical_middle;

		split_blob(self, root->next[0], tlc_blob, image, width, height);

		/* top right corner. */
		trc_blob.top = quad.top;
		trc_blob.left = vertical_middle;
		trc_blob.bottom = horizon_middle;
		trc_blob.right = quad.right;
		
		split_blob(self, root->next[1], trc_blob, image, width, height);
		
		/* lower left corner. */
		llc_blob.top = horizon_middle;
		llc_blob.left = quad.left;
		llc_blob.bottom = quad.bottom;
		llc_blob.right = vertical_middle;
		
		split_blob(self, root->next[2], llc_blob, image, width, height);
		
		/*lower right corner.*/
		lrc_blob.top = horizon_middle;
		lrc_blob.left = vertical_middle;
		lrc_blob.bottom = quad.bottom;
		lrc_blob.right = quad.right;
		
		split_blob(self, root->next[3], lrc_blob, image, width, height);
	}
}

/** @brief Add node to tree.
 ** @param self quadtree instance.
 ** @param blob image blob.
 ** @return added node.
 **/
QNode *add_node(QTree *self, Blob *blob)
{
	QNode *child = NULL;
	
	child = (QNode *)malloc(sizeof(QNode));
	assert(child);
	
	child->blob.quad.top = blob->quad.top;
	child->blob.quad.left = blob->quad.left;
	child->blob.quad.bottom = blob->quad.bottom;
	child->blob.quad.right = blob->quad.right;
	child->blob.range = blob->range;
	child->next[0] = NULL;
	child->next[1] = NULL;
	child->next[2] = NULL;
	child->next[3] = NULL;
	
	if (!self->root) {
		self->root = child;
	} else {
		add_child_node(self->root, child);
	}
	
	self->depth++;
	
	return child;
}

/** @brief Add child node to father node.
 ** @param root father node.
 ** @param node child node.
 **/
void add_child_node(QNode *root, QNode *node)
{
	int child_id;
	
	child_id = which_child_of_father(&root->blob, &node->blob);
	if (-1 == child_id) {
		return;
	}
	
	switch (child_id) {
	case TLC:
	{
		if (!root->next[0]) {
			root->next[0] = node;
		} else {
			add_child_node(root->next[0], node);
		}
	}
		break;
	case TRC:
	{
		if (!root->next[1]) {
			root->next[1] = node;
		} else {
			add_child_node(root->next[1], node);
		}
	}
		break;
	case LLC:
	{
		if (!root->next[2]) {
			root->next[2] = node;
		} else {
			add_child_node(root->next[2], node);
		}
	}
		break;
	case LRC:
	{
		if (!root->next[3]) {
			root->next[3] = node;
		} else {
			add_child_node(root->next[3], node);
		}
	}
		break;
	default:
		break;
	}
}

/** @brief Which child of father.
 ** @param father father blob.
 ** @param child child blob.
 ** @return child identity if found,
 **            -1 if not found.
 **/
int which_child_of_father(const Blob *father, const Blob *child)
{
	unsigned int horizon_middle;
	unsigned int vertical_middle;
	
	horizon_middle = (father->quad.top + father->quad.bottom) >> 1;
	vertical_middle = (father->quad.left + father->quad.right) >> 1;
	
	if (child->quad.left >= father->quad.left &&
		child->quad.right <= vertical_middle &&
		child->quad.top >= father->quad.top &&
		child->quad.bottom <= horizon_middle) {
		return TLC;
	} else if (child->quad.left >= vertical_middle &&
		child->quad.right <= father->quad.right &&
		child->quad.top >= father->quad.top &&
		child->quad.bottom <= horizon_middle) {
		return TRC;
	} else if (child->quad.left >= father->quad.left &&
		child->quad.right <= vertical_middle &&
		child->quad.top >= horizon_middle &&
		child->quad.bottom <= father->quad.bottom) {
		return LLC;
	} else if (child->quad.left >= vertical_middle &&
		child->quad.right <= father->quad.right &&
		child->quad.top >= horizon_middle &&
		child->quad.bottom <= father->quad.bottom) {
		return LRC;
	} else {
		return -1;
	}
}

/** @brief Get leaf node fo quadtree.
 ** @param root root node.
 ** @param nblobs number of blobs.
 **/
void get_leafnode(QNode *root, Blob *blobs, int *nblobs)
{	
	if (!root) {
		return;
	}

	if (NULL == root->next[0] &&
		NULL == root->next[1] &&
		NULL == root->next[2] &&
		NULL == root->next[3]) {
		blobs[*nblobs].quad.top = root->blob.quad.top;
		blobs[*nblobs].quad.left = root->blob.quad.left;
		blobs[*nblobs].quad.bottom = root->blob.quad.bottom;
		blobs[*nblobs].quad.right = root->blob.quad.right;
		blobs[*nblobs].range = root->blob.range;
		(*nblobs)++;
	}
	
	if (root->next[0]) {
		get_leafnode(root->next[0], blobs, nblobs);
	}
	
	if (root->next[1]) {
		get_leafnode(root->next[1], blobs, nblobs);
	}
	
	if (root->next[2]) {
		get_leafnode(root->next[2], blobs, nblobs);
	}
	
	if (root->next[3]) {
		get_leafnode(root->next[3], blobs, nblobs);
	}
}

/** @brief Delete all nodes.
 ** @param root root node.
 **/
void delete_all_nodes(QNode *root)
{
	QNode *second_node;
	QNode *third_node;
	QNode *fourth_node;
	
	if (root) {
		second_node = root->next[1];
		third_node = root->next[2];
		fourth_node = root->next[3];
		
		delete_all_nodes(root->next[0]);
	
		free(root);
		
		delete_all_nodes(second_node);
		delete_all_nodes(third_node);
		delete_all_nodes(fourth_node);
	}
}