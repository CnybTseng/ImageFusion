/** @file fusion.c - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <windows.h>

#include "fusion.h"
#include "pthread.h"
#include "fifo.h"
#include "registration.h"
#include "bkgreconstruct.h"
#include "imgsubtract.h"
#include "imgadd.h"
#include "imgmul.h"
#include "RDC.h"

typedef enum
{
	FRAME_RESOLUTION_OF_384 = 15,
	FRAME_RESOLUTION_OF_640 = 16,
	PIXEL_FORMAT_YUV_SEMIPLANAR_422 = 22,
	PIXEL_FORMAT_YUV_SEMIPLANAR_420 = 23,
}RDC_Sets;

typedef enum
{
	GRAY_STYLE = 0,
	COLOR_STYLE
}FusionColor;

struct tagFusion
{
	int caches;						/**< image caches. */
	int base_width;					/**< width of base image. */
	int base_height;				/**< height of base image. */
	int unreg_width;				/**< width of unregistered image. */
	int unreg_height;				/**< height of unregistered image. */
	int rawi_image_size;			/**< raw infrared image size. */
	int rawv_image_size;			/**< raw visual image size. */
	int yuvf_image_size;			/**< YUV420 format image size. */
	int nmsc_image_size;			/**< normalized single channel image size. */
	int *contrl_points;				/**< control points. */
	int npoints;					/**< number of control points. */
	int ngls;						/**< number of unsuppression fusion image gray levels. */
	float ssr;						/**< standard bright feature suppression ratio. */
	float bpr;						/**< brightest pixel ratio. */
	Fifo *rawi_ring;				/**< raw infrared image ring buffer. */
	Fifo *rawv_ring;				/**< raw visual image ring buffer. */
	Fifo *gsci_ring;				/**< infrared gray scale compressed image ring buffer. */
	Fifo *regt_ring;				/**< visual registered image ring buffer. */
	Fifo *fusn_ring;				/**< fusion image ring buffer. */
	Fifo *iout_ring;				/**< infrared image output queue. */
	Fifo *vout_ring;				/**< visual image output queue. */
	Fifo *brft_ring;				/**< bright feature output queue. */
	Registration *regist;			/**< image registration instance. */
	BkgReconst *breconst;			/**< background reconstruction instance. */
	RDC_Sets rdc_reso;				/**< RDC resolution set. */
	RDC_Sets rdc_out_format;		/**< RDC output format set. */
	FusionColor cstyle;				/**< color style of fusion image. */
	unsigned int *hist;				/**< histogram of unsuppression fusion image. */
	unsigned char *i_rawi_image;	/**< input raw infrared image. */
	unsigned char *i_rawv_image;	/**< input raw visual image. */
	unsigned char *o_rawi_image;	/**< output raw infrared image. */
	unsigned char *o_rawv_image;	/**< output raw visual image. */
	unsigned char *i_gsci_image;	/**< input infrared gray scale compressed image. */
	unsigned char *i_regt_image;	/**< input visual registered image. */
	unsigned char *o_gsci_image;	/**< output infrared gray scale compressed image. */
	unsigned char *o_regt_image;	/**< output visual registered image. */
	unsigned char *bkgr_image;		/**< background reconstruction image. */
	unsigned char *etbk_image;		/**< estimated background image. */
	unsigned char *brft_image;		/**< bright feature image. */
	unsigned char *rfbf_image;		/**< refine bright feature image. */
	unsigned char *sbrf_image;		/**< suppressed bright feature image. */
	unsigned short *usfn_image;		/**< unsuppression fusion image. */
	unsigned char *i_fusn_image;	/**< input fusion image. */
	unsigned char *o_fusn_image;	/**< output fusion image. */
	unsigned char *iout_image;		/**< infrared output image. */
	unsigned char *vout_image;		/**< visual output image. */
	unsigned char *fout_image;		/**< feature output image. */
	int stop_fusn;					/**< fusion thread state. */
};

/** @name some private functions
 ** @{ */
static int get_text_lines(const char *filename);
static unsigned int roundup_power_of_2(unsigned int a);
static void *fusion_thread(void *s);
static int preprocess_infrared_start(Fusion *self);
static void *preprocess_infrared_thread(void *s);
static int preprocess_visual_start(Fusion *self);
static void *preprocess_visual_thread(void *s);
static void suppress_bright_feature(const unsigned char *rfbf_image, unsigned int width,
                                    unsigned int height, unsigned short *usfn_image,
									unsigned int *hist, unsigned int ngls, float ssr,
									float bpr, unsigned char *sbrf_image);
/** @} */

/** @brief Create a new instance of image fusion.
 ** @return the new instance.
 **/
Fusion *fusion_new()
{
	Fusion *self = (Fusion *)malloc(sizeof(Fusion));
	return self;
}

/** @brief Initialize fusion instance.
 ** @param self fusion instance.
 ** @param base_width width of base image.
 ** @param base_height height of base image.
 ** @param unreg_width width of unregistered image.
 ** @param unreg_height height of unregistered image.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int fusion_init(Fusion *self,
                int base_width, int base_height,
                int unreg_width, int unreg_height)
{
	assert(self);
	
	self->caches = 4;
	self->base_width = base_width;
	self->base_height = base_height;
	self->unreg_width = unreg_width;
	self->unreg_height = unreg_height;
	self->rawi_image_size = base_width * base_height * sizeof(unsigned short);
	self->rawi_image_size = roundup_power_of_2(self->rawi_image_size);
	self->rawv_image_size = unreg_width * unreg_height * 3 >> 1;
	self->rawv_image_size = roundup_power_of_2(self->rawv_image_size);
	self->yuvf_image_size = base_width * base_height * 3 >> 1;
	self->yuvf_image_size = roundup_power_of_2(self->yuvf_image_size);
	self->nmsc_image_size = base_width * base_height;
	self->nmsc_image_size = roundup_power_of_2(self->nmsc_image_size);
	self->ngls = 0xFFFF + 1;
	self->npoints = get_text_lines("control_points.txt");
	self->ssr = 0.8f;
	self->bpr = 0.001f;
	self->rdc_reso = FRAME_RESOLUTION_OF_640;
	self->rdc_out_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
	self->cstyle = COLOR_STYLE;
	self->stop_fusn = 0;
	
	self->contrl_points = (int *)malloc(self->npoints * sizeof(int) * 2);
	if (!self->contrl_points) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->rawi_ring = fifo_alloc(self->caches * self->rawi_image_size);
	if (!self->rawi_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->rawv_ring = fifo_alloc(self->caches * self->rawv_image_size);
	if (!self->rawv_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->gsci_ring = fifo_alloc(self->caches * self->yuvf_image_size);
	if (!self->gsci_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->regt_ring = fifo_alloc(self->caches * self->yuvf_image_size);
	if (!self->regt_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->fusn_ring = fifo_alloc(self->caches * self->yuvf_image_size);
	if (!self->fusn_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->iout_ring = fifo_alloc(self->caches * self->yuvf_image_size);
	if (!self->iout_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->vout_ring = fifo_alloc(self->caches * self->yuvf_image_size);
	if (!self->vout_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->brft_ring = fifo_alloc(self->caches * self->yuvf_image_size);
	if (!self->brft_ring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->regist = rm_regist_new();
	if (!self->regist) {
		fprintf(stderr, "rm_regist_new fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
		
	if (rm_regist_init(self->regist, base_width, base_height, unreg_width,
		unreg_height, self->contrl_points, self->npoints, "interpY.txt", "interpX.txt")) {
		fprintf(stderr, "rm_regist_init fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->breconst = bkgreconst_new();
	if (!self) {
		fprintf(stderr, "bkgreconst_new fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	if (bkgreconst_init(self->breconst, base_width, base_height)) {
		fprintf(stderr, "bkgreconst_init fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	if (RDC_Init(self->rdc_out_format, self->rdc_reso)) {
		fprintf(stderr, "RDC_Init fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->hist = (unsigned int *)malloc(self->ngls * sizeof(unsigned int));
	if (!self->hist) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->i_rawi_image = (unsigned char *)malloc(self->rawi_image_size);
	if (!self->i_rawi_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->i_rawv_image = (unsigned char *)malloc(self->rawv_image_size);
	if (!self->i_rawv_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->o_rawi_image = (unsigned char *)malloc(self->rawi_image_size);
	if (!self->o_rawi_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->o_rawv_image = (unsigned char *)malloc(self->rawv_image_size);
	if (!self->o_rawv_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
		
	self->i_gsci_image = (unsigned char *)malloc(self->yuvf_image_size);
	if (!self->i_gsci_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->i_regt_image = (unsigned char *)malloc(self->yuvf_image_size);
	if (!self->i_regt_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->o_gsci_image = (unsigned char *)malloc(self->yuvf_image_size);
	if (!self->o_gsci_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->o_regt_image = (unsigned char *)malloc(self->yuvf_image_size);
	if (!self->o_regt_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->bkgr_image = (unsigned char *)malloc(self->nmsc_image_size);
	if (!self->bkgr_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->etbk_image = (unsigned char *)malloc(self->nmsc_image_size);
	if (!self->etbk_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->brft_image = (unsigned char *)malloc(self->nmsc_image_size);
	if (!self->brft_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
		
	self->rfbf_image = (unsigned char *)malloc(self->nmsc_image_size);
	if (!self->rfbf_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->sbrf_image = (unsigned char *)malloc(self->nmsc_image_size);
	if (!self->sbrf_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->usfn_image = (unsigned short *)malloc(base_width * base_height * sizeof(unsigned short));
	if (!self->usfn_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->i_fusn_image = (unsigned char *)malloc(self->yuvf_image_size);
	if (!self->i_fusn_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->o_fusn_image = (unsigned char *)malloc(self->yuvf_image_size);
	if (!self->o_fusn_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->iout_image = (unsigned char *)malloc(self->yuvf_image_size);
	if (!self->iout_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->vout_image = (unsigned char *)malloc(self->yuvf_image_size);
	if (!self->vout_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->fout_image = (unsigned char *)malloc(self->yuvf_image_size);
	if (!self->fout_image) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		clean:fusion_delete(self);
		return -1;
	}
	
	return 0;
}

/** @brief Delete fusion instance.
 ** @param self fusion instance.
 **/
void fusion_delete(Fusion *self)
{
	if (self) {
		if (self->rawi_ring) {
			fifo_delete(self->rawi_ring);
		}
		if (self->rawv_ring) {
			fifo_delete(self->rawv_ring);
		}
		if (self->gsci_ring) {
			fifo_delete(self->gsci_ring);
		}
		if (self->regt_ring) {
			fifo_delete(self->regt_ring);
		}
		if (self->fusn_ring) {
			fifo_delete(self->fusn_ring);
		}
		if (self->iout_ring) {
			fifo_delete(self->iout_ring);
		}
		if (self->vout_ring) {
			fifo_delete(self->vout_ring);
		}
		if (self->brft_ring) {
			fifo_delete(self->brft_ring);
		}
		if (self->regist) {
			rm_regist_delete(self->regist);
		}
		if (self->breconst) {
			bkgreconst_delete(self->breconst);
		}
		if (self->hist) {
			free(self->hist);
			self->hist = NULL;
		}
		if (self->i_rawi_image) {
			free(self->i_rawi_image);
			self->i_rawi_image = NULL;
		}
		if (self->i_rawv_image) {
			free(self->i_rawv_image);
			self->i_rawv_image = NULL;
		}
		if (self->o_rawi_image) {
			free(self->o_rawi_image);
			self->o_rawi_image = NULL;
		}
		if (self->o_rawv_image) {
			free(self->o_rawv_image);
			self->o_rawv_image = NULL;
		}
		if (self->i_gsci_image) {
			free(self->i_gsci_image);
			self->i_gsci_image = NULL;
		}
		if (self->i_regt_image) {
			free(self->i_regt_image);
			self->i_regt_image = NULL;
		}
		if (self->o_gsci_image) {
			free(self->o_gsci_image);
			self->o_gsci_image = NULL;
		}
		if (self->o_regt_image) {
			free(self->o_regt_image);
			self->o_regt_image = NULL;
		}
		if (self->bkgr_image) {
			free(self->bkgr_image);
			self->bkgr_image = NULL;
		}
		if (self->etbk_image) {
			free(self->etbk_image);
			self->etbk_image = NULL;
		}
		if (self->brft_image) {
			free(self->brft_image);
			self->brft_image = NULL;
		}
		if (self->rfbf_image) {
			free(self->rfbf_image);
			self->rfbf_image = NULL;
		}
		if (self->sbrf_image) {
			free(self->sbrf_image);
			self->sbrf_image = NULL;
		}
		if (self->usfn_image) {
			free(self->usfn_image);
			self->usfn_image = NULL;
		}
		if (self->i_fusn_image) {
			free(self->i_fusn_image);
			self->i_fusn_image = NULL;
		}
		if (self->o_fusn_image) {
			free(self->o_fusn_image);
			self->o_fusn_image = NULL;
		}
		if (self->iout_image) {
			free(self->iout_image);
			self->iout_image = NULL;
		}
		if (self->vout_image) {
			free(self->vout_image);
			self->vout_image = NULL;
		}
		if (self->fout_image) {
			free(self->fout_image);
			self->fout_image = NULL;
		}
		if (self) {
			free(self);
			self = NULL;
		}
	}
}

/** @brief Start image fusion thread.
 ** @param self fusion instance.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int fusion_start(Fusion *self)
{
	pthread_t tid;
	pthread_attr_t attr;
	int ret;
	
	assert(self);
	
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	ret = pthread_create(&tid, &attr, fusion_thread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	
	if (preprocess_infrared_start(self)) {
		self->stop_fusn = 1;
		return -1;
	}
	
	if (preprocess_visual_start(self)) {
		self->stop_fusn = 1;
		return -1;
	}
	
	if (bkgreconst_start(self->breconst)) {
		self->stop_fusn = 1;
		return -1;
	}
	
	return 0;
}

/** @brief Stop image fusion thread.
 ** @param self fusion instance.
 **/
void fusion_stop(Fusion *self)
{
	assert(self);
	self->stop_fusn = 1;
	bkgreconst_stop(self->breconst);
}

/** @brief Send image pair to fusion instance.
 ** @param self fusion instance.
 ** @param base base image.
 ** @param unreg unregistered image.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int fusion_put(Fusion *self, unsigned char *base, unsigned char *unreg)
{
	int len;
	int ret = 0;
	
	assert(self);
	assert(base);
	assert(unreg);
	
	memmove(self->i_rawi_image, base, self->base_width * self->base_height * sizeof(unsigned short));
	len = fifo_put(self->rawi_ring, self->i_rawi_image, self->rawi_image_size);
	if (len != self->rawi_image_size) {
		fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		ret = -1;
	}
	
	memmove(self->i_rawv_image, unreg, self->unreg_width * self->unreg_height * 3 >> 1);
	len = fifo_put(self->rawv_ring, self->i_rawv_image, self->rawv_image_size);
	if (len != self->rawv_image_size) {
		fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		ret = -1;
	}

	return ret;
}

/** @brief Send infrared image to fusion instance.
 ** @param self fusion instance.
 ** @param base base image.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int fusion_put_inf(Fusion *self, unsigned char *base)
{
	int len;
	
	assert(self);
	assert(base);
	
	memmove(self->i_rawi_image, base, self->base_width * self->base_height * sizeof(unsigned short));
	len = fifo_put(self->rawi_ring, self->i_rawi_image, self->rawi_image_size);
	if (len != self->rawi_image_size) {
		fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Send visual image to fusion instance.
 ** @param self fusion instance.
 ** @param unreg unregistered image.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int fusion_put_vis(Fusion *self, unsigned char *unreg)
{
	int len;
	int ret = 0;
	
	assert(self);
	assert(unreg);
	
	memmove(self->i_rawv_image, unreg, self->unreg_width * self->unreg_height * 3 >> 1);
	len = fifo_put(self->rawv_ring, self->i_rawv_image, self->rawv_image_size);
	if (len != self->rawv_image_size) {
		fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Get fusion image from fusion instance.
 ** @param self fusion instance.
 ** @param fu fusion image.
 ** @return 1 if success,
 **         0 if fail.
 **/
int fusion_get(Fusion *self, unsigned char *fu)
{
	int len;
	int ret;
	
	assert(self);
	assert(fu);
	
	len = self->yuvf_image_size;
	
	ret = fifo_get(self->fusn_ring, self->o_fusn_image, len);
	if (ret == len) {
		memmove(fu, self->o_fusn_image, self->base_width * self->base_height * 3 >> 1);
	}
	
	return (ret == len);
}

/** @brief Get grayscale compressed infrared image from fusion instance.
 ** @param self fusion instance.
 ** @param inf infrared image.
 ** @return 1 if success,
 **         0 if fail.
 **/
int fusion_get_inf(Fusion *self, unsigned char *inf)
{
	int len;
	int ret;
	
	assert(self);
	assert(inf);
	
	len = self->yuvf_image_size;
	
	ret = fifo_get(self->iout_ring, self->iout_image, len);
	if (ret == len) {
		memmove(inf, self->iout_image, self->base_width * self->base_height * 3 >> 1);
	}
	
	return (ret == len);
}

/** @brief Get registered visual image from fusion instance.
 ** @param self fusion instance.
 ** @param vis visual image.
 ** @return 1 if success,
 **         0 if fail.
 **/
int fusion_get_vis(Fusion *self, unsigned char *vis)
{
	int len;
	int ret;
	
	assert(self);
	assert(vis);
	
	len = self->yuvf_image_size;
	
	ret = fifo_get(self->vout_ring, self->vout_image, len);
	if (ret == len) {
		memmove(vis, self->vout_image, self->base_width * self->base_height * 3 >> 1);
	}
	
	return (ret == len);
}

/** @brief Get infrared bright feature from fusion instance.
 ** @param self fusion instance.
 ** @param ibf infrared bright feature image.
 ** @return 1 if success,
 **         0 if fail.
 **/
int fusion_get_ibf(Fusion *self, unsigned char *ibf)
{
	int len;
	int ret;
	
	assert(self);
	assert(ibf);
	
	len = self->nmsc_image_size;
	
	ret = fifo_get(self->brft_ring, self->fout_image, len);
	if (ret == len) {
		memmove(ibf, self->fout_image, self->base_width * self->base_height);
	}
	
	return (ret == len);
}

/** @brief Get text lines.
 ** @param filename text filename
 ** @return  lines if success,
 **             -1 if fail.
 **/
int get_text_lines(const char *filename)
{
	FILE *fp;
	int ch;
	int lines;
	
	fp = fopen(filename, "r");
	if (!fp) {
		return -1;
	}
	
	lines = 0;
	
	while ((ch = fgetc(fp)) != EOF) {
		if (ch == '\n') {
			lines++;
		}
	}
	
	fclose(fp);
	
	return lines;
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

/** @brief Image fusion thread.
 ** @param s fusion instance.
 **/
void *fusion_thread(void *s)
{
	Fusion *self = (Fusion *)s;
	int read_len;
	int write_len;
	
	while (!self->stop_fusn) {
		/* read infrared image from ring buffer. */
		read_len = fifo_get(self->gsci_ring, (char *)self->o_gsci_image, self->yuvf_image_size);
		if (read_len != self->yuvf_image_size) {
			continue;
		}

		/* read visual image from ring buffer. */
		read_len = fifo_get(self->regt_ring, (char *)self->o_regt_image, self->yuvf_image_size);
		if (read_len != self->yuvf_image_size) {
			continue;
		}

		/* read infrared reconstructed background from ring buffer. */
		if (!bkgreconst_get(self->breconst, self->bkgr_image)) {
			continue;
		}
		
		/* extract bright feature. */
		img_subtract_kr(self->o_gsci_image, self->base_width, self->base_height,
			self->bkgr_image, self->brft_image);
		
		/* estimate infrared background. */
		img_subtract_kr(self->o_regt_image, self->base_width, self->base_height,
			self->o_gsci_image, self->etbk_image);
		
		/* refine bright feature. */
		img_subtract_kr(self->brft_image, self->base_width, self->base_height,
			self->etbk_image, self->rfbf_image);
		
		/* make unsuppression fusion image. */
		img_add(self->o_regt_image, self->base_width, self->base_height, self->rfbf_image,
			self->usfn_image);
		
		/* suppress bright feature. */
		suppress_bright_feature(self->rfbf_image, self->base_width, self->base_height,
			self->usfn_image, self->hist, self->ngls, self->ssr, self->bpr, self->sbrf_image);
				
		/* overlay bright feature. */
		img_add_kr(self->o_regt_image, self->base_width, self->base_height, self->sbrf_image,
			self->i_fusn_image);

		if (COLOR_STYLE == self->cstyle) {
			memmove(self->i_fusn_image + self->base_width * self->base_height, self->o_regt_image +
				self->base_width * self->base_height, self->base_width * self->base_height >> 1);
		} else {
			memset(self->i_fusn_image + self->base_width * self->base_height, 0x80,
				self->base_width * self->base_height >> 1);
		}
			
		write_len = fifo_put(self->fusn_ring, (char *)self->i_fusn_image, self->yuvf_image_size);
		if (write_len != self->yuvf_image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
		write_len = fifo_put(self->brft_ring, self->sbrf_image, self->nmsc_image_size);
		if (write_len != self->nmsc_image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
	}
	
	return (void *)(0);
}

int preprocess_infrared_start(Fusion *self)
{
	pthread_t tid;
	pthread_attr_t attr;
	int ret;
	
	assert(self);
	
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	ret = pthread_create(&tid, &attr, preprocess_infrared_thread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	
	return 0;
}

void *preprocess_infrared_thread(void *s)
{
	Fusion *self = (Fusion *)s;
	int read_len;
	int write_len;
	unsigned int rol;
	
	while (!self->stop_fusn) {
		read_len = fifo_get(self->rawi_ring, self->o_rawi_image, self->rawi_image_size);
		if (read_len != self->rawi_image_size) {
			continue;
		}
		
		RDC_SendRawData(self->o_rawi_image, self->base_width * self->base_height *
			sizeof(unsigned short));
		RDC_GetFrame(self->i_gsci_image, &rol);
		
		bkgreconst_put(self->breconst, self->i_gsci_image);
		
		write_len = fifo_put(self->gsci_ring, (char *)self->i_gsci_image, self->yuvf_image_size);
		if (write_len != self->yuvf_image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
		write_len = fifo_put(self->iout_ring, (char *)self->i_gsci_image, self->yuvf_image_size);
		if (write_len != self->yuvf_image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
	}
	
	return (void *)(0);
}

int preprocess_visual_start(Fusion *self)
{
	pthread_t tid;
	pthread_attr_t attr;
	int ret;
	
	assert(self);
	
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	ret = pthread_create(&tid, &attr, preprocess_visual_thread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	
	return 0;
}

void *preprocess_visual_thread(void *s)
{
	Fusion *self = (Fusion *)s;
	int read_len;
	int write_len;
	
	while (!self->stop_fusn) {
		read_len = fifo_get(self->rawv_ring, self->o_rawv_image, self->rawv_image_size);
		if (read_len != self->rawv_image_size) {
			continue;
		}
		
		rm_regist_warp_image(self->regist, self->o_rawv_image, self->i_regt_image);
		
		write_len = fifo_put(self->regt_ring, self->i_regt_image, self->yuvf_image_size);
		if (write_len != self->yuvf_image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
		write_len = fifo_put(self->vout_ring, self->i_regt_image, self->yuvf_image_size);
		if (write_len != self->yuvf_image_size) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
	}
	
	return (void *)(0);
}

/** @brief Suppress infrared bright feature.
 ** @param rfbf_image refined bright feature image.
 ** @param width image width.
 ** @param height image height.
 ** @param usfn_image unsuppressed fusion image.
 ** @param hist image histogram.
 ** @param ngls number of unsuppressed fusion image gray levels.
 ** @param ssr standard bright feature suppression ratio.
 ** @param bpr brightest pixel ratio.
 ** @param sbrf_image suppressed bright feature image.
 **/
void suppress_bright_feature(const unsigned char *rfbf_image, unsigned int width,
                             unsigned int height, unsigned short *usfn_image,
							 unsigned int *hist, unsigned int ngls, float ssr,
							 float bpr, unsigned char *sbrf_image)
{
	unsigned int i;
	unsigned int npixels;
	unsigned int bpc = 0;	/* brightest pixel counter. */
	unsigned int bpct;		/* brightest pixel count threshold. */
	float sum = 0;
	float mean;
	float sr;				/* suppression ratio. */

	npixels = width * height;
	bpct = (unsigned int)(bpr * npixels);
	
	memset(hist, 0, ngls * sizeof(unsigned int));
	
	for (i = 0; i < npixels; i++) {
		hist[usfn_image[i]]++;
	}
	
	for (i = ngls - 1; i >= 0; i--) {
		if (!hist[i]) {
			continue;
		}
		
		bpc += hist[i];
		sum += hist[i] * i;
		if (bpc > bpct) {
			break;
		}
	}
	
	mean = sum / bpc;
	sr = 255 / mean;
	if (sr > ssr) {
		sr = ssr;
	}
	
	img_mul_s_kr(rfbf_image, width, height, sr, sbrf_image);
}