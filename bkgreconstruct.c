/** @file bkgreconstruct.c - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "bkgreconstruct.h"
#include "pthread.h"
#include "quadtree.h"
#include "fifo.h"
#include "minfilter.h"
#include "gaussfilter.h"

struct tagBkgReconst
{
	int caches;						/**< image caches. */
	unsigned int width;				/**< image width. */
	unsigned int height;			/**< image height. */
	unsigned int minbw;				/**< minimum blob width. */
	unsigned int minbh;				/**< minimum blob height. */
	unsigned int mingr;				/**< minimum gray range. */
	unsigned int mnbpi;				/**< maximum number of blobs per image. */
	unsigned int mf_size;			/**< minimum filter size. */
	unsigned int gf_size;			/**< gaussian filter size. */
	float gf_sigma;					/**< sigma of gaussian filter. */
	unsigned int image_size;		/**< image size. */
	unsigned int blob_size;			/**< blob maximum size per image. */
	Fifo *infd_ring;				/**< infrared image ring buffer.*/
	Fifo *infm_ring;				/**< infrared image ring buffer.*/
	Fifo *blob_ring;				/**< decomposed blob ring buffer. */
	Fifo *minf_ring;				/**< minimum filtered image ring buffer. */
	Fifo *gfbr_ring;				/**< gaussian filtered background ring buffer. */
	QTree *qtree;					/**< quadtree. */
	Blob *iblob;					/**< decomposed blobs. */
	Blob *oblob;					/**< output blob. */
	unsigned char *i_infd_image;	/**< input infrared image. */
	unsigned char *o_infd_image;	/**< output infrared image. */
	unsigned char *infm_image;		/**< infrared image for minimum filter. */
	unsigned char *i_minf_image;	/**< input minimum filter image. */
	unsigned char *o_minf_image;	/**< output minimum filter image. */
	unsigned char *bkgr_image;		/**< background reconstructed image. */
	unsigned char *i_gfbr_image;	/**< input gaussian filtered background image. */
	unsigned char *o_gfbr_image;	/**< output gaussian filtered background image. */
	float *U;						/**< interpolation ratio. */
	float *VT;						/**< interpolation ratio. */
	float *temp1;					/**< temporary variable. */
	float *temp2;					/**< temporary variable. */
	int stop_reconst;				/**< reconstruction thread state. */
};

/** @name some private functions
 ** @{ */
static unsigned int roundup_power_of_2(unsigned int a);
static int minimum_filter_start(BkgReconst *self);
static void *minimum_filter_thread(void *s);
static int quadtree_decomp_start(BkgReconst *self);
static void *quadtree_decomp_thread(void *s);
static void bezier_interpolate(unsigned char *image, unsigned int width,
                               unsigned int height, Blob *blob, int nblobs,
							   float *U, float *VT, float *temp1, float *temp2,
							   unsigned char *bkgr_image);
static void bezier_interp_coeff(float *ic, int dimx, int dimy);
static void bezier_trans_matrix(float *a, int aw, int ah, float *b);
static void bezier_mul_matrix(const float *a, unsigned int aw, unsigned int ah,
                              const float *b, unsigned int bw, unsigned int bh, float *c);
static void bezier_cpoint_feature(unsigned char *image, unsigned int width,
                                  unsigned int height, Quadrant *quad,
						          float *feat, int nfeats);
static void bezier_set_surf(unsigned char *image, unsigned int width,
                            unsigned int height, Quadrant *quad, const float *surf,
							unsigned int dimx, unsigned int dimy);
static void *bkgreconst_thread(void *s);
/** @} */

/** @brief Create a new instance of BkgReconst.
 ** @return the new instance.
 **/
BkgReconst *bkgreconst_new()
{
	BkgReconst *self = (BkgReconst *)malloc(sizeof(BkgReconst));
	return self;
}

/** @brief Initialize new BkgReconst instance.
 ** @param self BkgReconst instance.
 ** @param width image width.
 ** @param height image height.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int bkgreconst_init(BkgReconst *self, unsigned int width,
                    unsigned int height)
{
	assert(self);
	
	self->caches = 8;
	self->width = width;
	self->height = height;
	self->minbw = 12;
	self->minbh = 9;
	self->mingr = 78;
	self->mnbpi = self->width * self->height / self->minbw / self->minbh;
	self->mf_size = 11;
	self->gf_size = 9;
	self->gf_sigma = 4.5f;
	self->image_size = self->width * self->height * sizeof(unsigned char);
	self->image_size = roundup_power_of_2(self->image_size);
	self->blob_size = self->mnbpi * sizeof(Blob);
	self->blob_size = roundup_power_of_2(self->blob_size);
	self->stop_reconst = 0;
	
	self->infd_ring = fifo_alloc(self->caches * self->image_size);
	if (!self->infd_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->infm_ring = fifo_alloc(self->caches * self->image_size);
	if (!self->infm_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->blob_ring = fifo_alloc(self->caches * self->blob_size);
	if (!self->blob_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->minf_ring = fifo_alloc(self->caches * self->image_size);
	if (!self->minf_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->gfbr_ring = fifo_alloc(self->caches * self->image_size);
	if (!self->gfbr_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->qtree = qtree_new();
	if (!self->qtree) {
		fprintf(stderr, "qtree_new fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	qtree_init(self->qtree, self->minbw, self->minbh, self->mingr);
	
	self->iblob = (Blob *)malloc(self->blob_size);
	if (!self->iblob) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->oblob = (Blob *)malloc(self->blob_size);
	if (!self->oblob) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->i_infd_image = (unsigned char *)malloc(self->image_size);
	if (!self->i_infd_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->o_infd_image = (unsigned char *)malloc(self->image_size);
	if (!self->o_infd_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->infm_image = (unsigned char *)malloc(self->image_size);
	if (!self->infm_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->i_minf_image = (unsigned char *)malloc(self->image_size);
	if (!self->i_minf_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->o_minf_image = (unsigned char *)malloc(self->image_size);
	if (!self->o_minf_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->bkgr_image = (unsigned char *)malloc(self->image_size);
	if (!self->bkgr_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->i_gfbr_image = (unsigned char *)malloc(self->image_size);
	if (!self->i_gfbr_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->o_gfbr_image = (unsigned char *)malloc(self->image_size);
	if (!self->o_gfbr_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->U = (float *)malloc(height * 4 * sizeof(float));
	if (!self->U) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->VT = (float *)malloc(4 * width * sizeof(float));
	if (!self->VT) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->temp1 = (float *)malloc(height * 4 * sizeof(float));
	if (!self->temp1) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->temp2 = (float *)malloc(height * width * sizeof(float));
	if (!self->temp2) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		clean:bkgreconst_delete(self);
		return -1;
	}
	
	return 0;
}					

/** @brief Delete BkgReconst instance.
 ** @param self BkgReconst instance.
 **/ 
void bkgreconst_delete(BkgReconst *self)
{
	if (self) {
		if (self->infd_ring) {
			fifo_delete(self->infd_ring);
		}
		if (self->infm_ring) {
			fifo_delete(self->infm_ring);
		}
		if (self->blob_ring) {
			fifo_delete(self->blob_ring);
		}
		if (self->minf_ring) {
			fifo_delete(self->minf_ring);
		}
		if (self->gfbr_ring) {
			fifo_delete(self->gfbr_ring);
		}
		if (self->qtree) {
			qtree_delete(self->qtree);
		}
		if (self->iblob) {
			free(self->iblob);
			self->iblob = NULL;
		}
		if (self->oblob) {
			free(self->oblob);
			self->oblob = NULL;
		}
		if (self->i_infd_image) {
			free(self->i_infd_image);
			self->i_infd_image = NULL;
		}
		if (self->o_infd_image) {
			free(self->o_infd_image);
			self->o_infd_image = NULL;
		}
		if (self->infm_image) {
			free(self->infm_image);
			self->infm_image = NULL;
		}
		if (self->i_minf_image) {
			free(self->i_minf_image);
			self->i_minf_image = NULL;
		}
		if (self->o_minf_image) {
			free(self->o_minf_image);
			self->o_minf_image = NULL;
		}
		if (self->bkgr_image) {
			free(self->bkgr_image);
			self->bkgr_image = NULL;
		}
		if (self->i_gfbr_image) {
			free(self->i_gfbr_image);
			self->i_gfbr_image = NULL;
		}
		if (self->o_gfbr_image) {
			free(self->o_gfbr_image);
			self->o_gfbr_image = NULL;
		}
		if (self->U) {
			free(self->U);
			self->U = NULL;
		}
		if (self->VT) {
			free(self->VT);
			self->VT = NULL;
		}
		if (self->temp1) {
			free(self->temp1);
			self->temp1 = NULL;
		}
		if (self->temp2) {
			free(self->temp2);
			self->temp2 = NULL;
		}
		
		free(self);
		self = NULL;
	}
}

/** @brief Start background reconstruction thread.
 ** @param self BkgReconst instance.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int bkgreconst_start(BkgReconst *self)
{
	pthread_t tid;
	pthread_attr_t attr;
	int ret;
	
	assert(self);
	
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	ret = pthread_create(&tid, &attr, bkgreconst_thread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	
	if (0 != minimum_filter_start(self)) {
		self->stop_reconst = 1;
		return -1;
	}
	
	if (0 != quadtree_decomp_start(self)) {
		self->stop_reconst = 1;
		return -1;
	}
	
	return 0;
}

/** @brief Stop background reconstruction thread.
 ** @param self BkgReconst instance.
 **/
void bkgreconst_stop(BkgReconst *self)
{
	assert(self);
	self->stop_reconst = 1;
}

/** @brief Send image to BkgReconst instance.
 ** @param self BkgReconst instance.
 ** @param image infrared image.
 ** @return 1 if success,
 **         0 if fail.
 **/
int bkgreconst_put(BkgReconst *self,
                   unsigned char *image)
{
	int len;
	int ret;
	
	assert(self);
	assert(image);
	
	len = self->image_size;
	
	memmove(self->i_infd_image, image, self->width * self->height * sizeof(unsigned char));
	ret = fifo_put(self->infd_ring, self->i_infd_image, len);
	if (ret != len) {
		fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		return 0;
	}
	
	ret = fifo_put(self->infm_ring, self->i_infd_image, len);
	if (ret != len) {
		fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		return 0;
	}
		
	return 1;
}				   
	
/** @brief Get reconstructed background from BkgReconst instance.
 ** @param self BkgReconst instance.
 ** @param bkg reconstructed background image.
 ** @return 1 if success,
 **         0 if fail.
 **/ 
int bkgreconst_get(BkgReconst *self,
                   unsigned char *bkg)
{
	int len;
	int ret;
	
	assert(self);
	assert(bkg);
	
	len = self->image_size;

	ret = fifo_get(self->gfbr_ring, self->o_gfbr_image, len);
	if (ret == len) {
		memmove(bkg, self->o_gfbr_image, self->width * self->height * sizeof(unsigned char));
	}
	
	return (ret == len);
}

/** @brief Round up to power of 2.
 ** @param a input number.
 ** @return a number rounded up to power of 2.
 **/
unsigned int roundup_power_of_2(unsigned int a)
{
	unsigned int position;
	int i;
	
	if (a == 0) {
		return 0;
	}

	position = 0;
	for (i = a; i != 0; i >>= 1) {
		position++;
	}

	return (unsigned int)(1 << position);
}

/** @brief Start minimum filter thread.
 ** @param self BkgReconst instance.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int minimum_filter_start(BkgReconst *self)
{
	pthread_t tid;
	pthread_attr_t attr;
	int ret;
	
	assert(self);
	
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	ret = pthread_create(&tid, &attr, minimum_filter_thread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	
	return 0;
}

/** @brief Minimum filter thread.
 ** @param s thread parameter.
 **/
void *minimum_filter_thread(void *s)
{
	BkgReconst *self = (BkgReconst *)s;
	int read_len;
	int write_len;
	
	while (!self->stop_reconst) {
		/* read infrared image from ring buffer. */
		read_len = fifo_get(self->infm_ring, self->infm_image, self->image_size);
		if (read_len != self->image_size) {
			continue;
		}
		
		/* minimum filter. */
		min_filter(self->infm_image, self->width, self->height, self->mf_size, self->i_minf_image);
		/*{
			FILE *fp = fopen("minf.dat", "wb");
			fwrite(self->i_minf_image, 1, self->width * self->height, fp);
			fclose(fp);
		}*/
		
		/* write filtered image to ring buffer. */
		write_len = fifo_put(self->minf_ring, self->i_minf_image, self->image_size);
		if (write_len != self->image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
	}
	
	return (void *)(0);
}

/** @brief Start quadtree decompose thread.
 ** @param self BkgReconst instance.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int quadtree_decomp_start(BkgReconst *self)
{
	pthread_t tid;
	pthread_attr_t attr;
	int ret;
	
	assert(self);
	
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	ret = pthread_create(&tid, &attr, quadtree_decomp_thread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	
	return 0;
}

/** @brief Quadtree decompose thread.
 ** @param s thread parameter.
 **/
void *quadtree_decomp_thread(void *s)
{
	BkgReconst *self = (BkgReconst *)s;
	int read_len;
	int nblobs;
	int write_len;
	
	while (!self->stop_reconst) {
		/* read infrared image from ring buffer. */
		read_len = fifo_get(self->infd_ring, self->o_infd_image, self->image_size);
		if (read_len != self->image_size) {
			continue;
		}

		/* quadtree decompose. */
		qtree_decompose(self->qtree, self->o_infd_image, self->width, self->height);
		
		memset(self->iblob, 0, self->mnbpi * sizeof(Blob));
		nblobs = gtree_get_leafnode(self->qtree, self->iblob);
		if (nblobs <= 0) {
			fprintf(stderr, "gtree_get_leafnode fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
		qtree_reset(self->qtree);

		/* write decomposed blobs to ring buffer. */
		write_len = fifo_put(self->blob_ring, (char *)self->iblob, self->blob_size);
		if (write_len != self->blob_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
	}
	
	return (void *)s;
}

/** @brief Bezier interpolation.
 ** @param image infrared image.
 ** @param width image width.
 ** @param height image height.
 ** @param blob decomposed blob.
 ** @param nblobs number of decomposed blobs.
 ** @param U interpolation coefficient matrix in Y direction.
 ** @param VT interpolation coefficient matrix in X direction.
 ** @param temp1 temporary matrix.
 ** @param temp2 temporary matrix.
 ** @param bkgr_image background reconstructed image. 
 **/
void bezier_interpolate(unsigned char *image, unsigned int width,
                        unsigned int height, Blob *blob, int nblobs,
						float *U, float *VT, float *temp1, float *temp2,
						unsigned char *bkgr_image)
{
	int i;
	unsigned int dimx, dimy;
	Blob *bptr = blob;
	float P[16];
	
	/* constant interpolation coefficient matrix. */
	const float M[] = { 1,  0,  0,  0,
	                   -3,  3,  0,  0,
					    3, -6,  3,  0,
					   -1,  3, -3,  1};
	
	/* transpose of matrix M. */
	const float MT[] = { 1, -3,  3, -1,
	                     0,  3, -6,  3,
					     0,  0,  3, -3,
					     0,  0,  0,  1};

	for (i = 0; i < nblobs; i++) {	
		if (bptr->quad.right <= 0 || bptr->quad.bottom <= 0) {
			break;
		}
	
		dimx = bptr->quad.right - bptr->quad.left;
		dimy = bptr->quad.bottom - bptr->quad.top;

		bezier_interp_coeff(U, 4, dimy);
		
		bezier_interp_coeff(temp2, 4, dimx);
		bezier_trans_matrix(temp2, 4, dimx, VT);

		bezier_cpoint_feature(image, width, height, &bptr->quad, P, 16);
		
		bezier_mul_matrix(U, 4, dimy, M, 4, 4, temp1);			/* U*M. */
		bezier_mul_matrix(temp1, 4, dimy, P, 4, 4, temp2);		/* U*M*P. */
		bezier_mul_matrix(temp2, 4, dimy, MT, 4, 4, temp1);		/* U*M*P*MT. */
		bezier_mul_matrix(temp1, 4, dimy, VT, dimx, 4, temp2);	/* U*M*P*MT*VT. */

		bezier_set_surf(bkgr_image, width, height, &bptr->quad, temp2, dimx, dimy);
		
		bptr++;
	}
}

/** @brief Calculate interpolation coefficient matrix.
 ** @param ic interpolation coefficient matrix.
 ** @param dimx dimension in X direction.
 ** @param dimy dimension in Y direction.
 **/
void bezier_interp_coeff(float *ic, int dimx, int dimy)
{
	int i;
	float val;
	float *icptr = NULL;
	
	for (i = 0; i < dimy; i++) {
		icptr = ic + i * dimx;
		val = i / (dimy - 1.0f);
		*(icptr) = 1;
		*(icptr + 1) = val;
		*(icptr + 2) = val * val;
		*(icptr + 3) = val * val * val;
	}
}

/** @brief Transpose matrix.
 ** @param a input matrix.
 ** @param aw dimension in X direction.
 ** @param ah dimension in Y direction.
 ** @param b transposed matrix.
 **/
void bezier_trans_matrix(float *a, int aw, int ah, float *b)
{
	int y, x;
	
	for (y = 0; y < aw; y++) {
		for (x = 0; x < ah; x++) {
			b[y * ah + x] = a[x * aw + y];
		}
	}
}

/** @brief Multiply matrix.
 ** @param a matrix a.
 ** @param aw width of matrix a.
 ** @param ah height of matrix b.
 ** @param b matrix b.
 ** @param bw width of matrix b.
 ** @param bh height of matrix b.
 ** @param c product of a*b.
 **/
void bezier_mul_matrix(const float *a, unsigned int aw, unsigned int ah,
                       const float *b, unsigned int bw, unsigned int bh, float *c)
{
	unsigned int x, y;
	unsigned int i;
	float sum;
	
	if (aw != bh) {
		fprintf(stderr, "bezier_mul_matrix fail[%s:%d].\n", __FILE__, __LINE__);
		return;
	}
	
	for (y = 0; y < ah; y++) {
		for (x = 0; x < bw; x++) {
			sum = 0;
			for (i = 0; i < aw; i++) {
				sum += a[y * aw + i] * b[i * bw + x];
			}
			
			c[y * bw + x] = sum;
		}
	}
}

/** @brief Extract control point feature of blob.
 ** @param image infrared image.
 ** @param width image width.
 ** @param height image height.
 ** @param quad quadrant of blob in image.
 ** @param feat extracted feature.
 ** @param nfeats number of features.
 **/
void bezier_cpoint_feature(unsigned char *image, unsigned int width,
                           unsigned int height, Quadrant *quad,
						   float *feat, int nfeats)
{
	unsigned int x, y;
	unsigned int bw, bh;
	const unsigned int fdim = 4;
	unsigned char *iptr = NULL;
	int i = 0;
	
	bw = quad->right - quad->left;
	bh = quad->bottom - quad->top;
	
	for (y = 0; y < fdim; y++) {
		iptr = image + width * (quad->top + (unsigned int)(y * bh / (float)fdim));
		for (x = 0; x < fdim; x++) {
			feat[i] = *(iptr + quad->left + (unsigned int)(x * bw / (float)fdim));
			i++;
		}
	}
}

/** @brief Copy reconstructed surface to background image.
 ** @param image background image.
 ** @param width image width.
 ** @param height image height.
 ** @param quad corresponding quadrant of reconstructed surface.
 ** @param surf reconstructed surface.
 ** @param dimx surface dimension in X direction.
 ** @param dimy surface dimension in Y direction.
 **/
void bezier_set_surf(unsigned char *image, unsigned int width,
                     unsigned int height, Quadrant *quad, const float *surf,
					 unsigned int dimx, unsigned int dimy)
{
	unsigned int x, y;
	unsigned char *iptr = NULL;
	float *sptr = NULL;
	unsigned int i = 0;
	
	iptr = image + quad->top * width;
	sptr = (float *)surf;
	
	for (y = quad->top; y < quad->bottom; y++) {
		for (x = quad->left; x < quad->right; x++) {
			*(iptr + x) = (unsigned char)(*(sptr + i));
			i++;
		}
		iptr += width;
	}
}

/** @brief Background reconstruction thread.
 ** @param s thread parameter.
 **/
void *bkgreconst_thread(void *s)
{
	BkgReconst *self = (BkgReconst *)s;
	int read_len;
	int write_len;
	
	while (!self->stop_reconst) {
		/* read minimum filtered image from ring buffer. */
		read_len = fifo_get(self->minf_ring, self->o_minf_image, self->image_size);
		if (read_len != self->image_size) {
			continue;
		}

		/* read quadtree decomposed blobs from ring buffer. */
		while (!self->stop_reconst) {
			read_len = fifo_get(self->blob_ring, (char *)self->oblob, self->blob_size);
			if (read_len == self->blob_size) {
				break;
			}
		}

		/* Bezier interpolation. */
		bezier_interpolate(self->o_minf_image, self->width, self->height, self->oblob,
			self->mnbpi, self->U, self->VT, self->temp1, self->temp2, self->bkgr_image);
				
		/* gaussian filter. */
		gauss_filter(self->bkgr_image, self->width, self->height, self->gf_sigma, self->i_gfbr_image);
			
		/* write reconstructed background to ring buffer. */
		write_len = fifo_put(self->gfbr_ring, self->i_gfbr_image, self->image_size);
		if (write_len != self->image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
	}
	
	return (void *)(0);
}