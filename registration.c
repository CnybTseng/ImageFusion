/** @file registration.c - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <io.h>

#include "registration.h"

/** @typedef RegistrationConst
 ** @brief Registration enumerate variables
 **/
typedef enum
{
	NAMELEN = 256,			/**< length of file name. */
	MIN_POINT_SIZE = 6,		/**< minimum control point number. */
}RegistrationConst;

struct tagRegistration
{
	int base_width;						/**< width of base image. */
	int base_height;					/**< height of base image. */
	int unreg_width;					/**< width of unregistered image. */
	int unreg_height;					/**< height of unregistered image. */
	float *row_inter_tab;				/**< row interpolation table. */
	float *col_inter_tab;				/**< col interpolation table. */
	float affine_matrix[6];				/**< affine matrix. */
};

/** @name Some private functions 
 ** @{ */
static int load_interp_table(const char *filename,
                             float *const tab, int rows, int cols);

static void cal_affine_matrix(const int *const contrl_points, int npoints,
                              float *const affine_matrix);

static void cal_interp_table(const float *const affine_matrix,
                             int base_width, int base_height,
							 int unreg_width, int unreg_height,
							 float *const row_inter_tab,
							 float *const col_inter_tab);

static void save_interp_table(const float *const tab,
							  int rows, int cols,
							  const char *filename);
/** @} */

/** @name Gauss elimination method.
 ** @} */
static void ge_swap_row(float *const mat, int order,
                        int rowa, int rowb);
						
static void ge_select_primary_element(float *const mat, int order);

static void ge_solver(float *const mat, int order);				 
/** @} */ 

/** @brief Create a new instance of registration.
 ** @return the new instance.
 **/ 
Registration *rm_regist_new()
{
	Registration *self = (Registration *)malloc(sizeof(Registration));
	return self;
}

/** @brief Initaialize registration.
 ** @param self registration instance.
 ** @param base_width width of base image.
 ** @param base_height height of base image.
 ** @param unreg_width width of unregistered image.
 ** @param unreg_height height of unregistered image.
 ** @param contrl_points control points.
 ** @param npoints number of control points.
 ** @param rtf row interpolation table filename.
 ** @param ctf column interpolation table filename.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int rm_regist_init(Registration *self,
                   int base_width, int base_height,
				   int unreg_width, int unreg_height,
                   const int *const contrl_points, int npoints,
                   const char *rtf, const char *ctf)
{	
	assert(self);
	assert(contrl_points);
	
	if (npoints < MIN_POINT_SIZE) {
		return -1;
	}
	
	self->base_width = base_width;
	self->base_height = base_height;
	self->unreg_width = unreg_width;
	self->unreg_height = unreg_height;
	
	self->row_inter_tab = (float *)malloc(base_width * base_height * sizeof(float));
	assert(self->row_inter_tab);
	
	self->col_inter_tab = (float *)malloc(base_width * base_height * sizeof(float));
	assert(self->col_inter_tab);
	
	if (load_interp_table(rtf, self->row_inter_tab, base_height, base_width) ||
		load_interp_table(ctf, self->col_inter_tab, base_height, base_width)) {		
		cal_affine_matrix(contrl_points, npoints, self->affine_matrix);
		cal_interp_table(self->affine_matrix, base_width, base_height,
			unreg_width, unreg_height, self->row_inter_tab, self->col_inter_tab);
		save_interp_table(self->row_inter_tab, base_height, base_width, rtf);
		save_interp_table(self->col_inter_tab, base_height, base_width, ctf);
	}
	
	return 0;
}

/** @brief Delete registration instance.
 ** @param self registration instance.
 **/
void rm_regist_delete(Registration *self)
{
	assert(self);
	
	if (self->row_inter_tab) {
		free(self->row_inter_tab);
		self->row_inter_tab = 0;
	}
	
	if (self->col_inter_tab) {
		free(self->col_inter_tab);
		self->col_inter_tab = 0;
	}
	
	free(self);
}

/** @brief Warp registration image.
 ** @param self registration instance.
 ** @param src source image.
 ** @param warped image.
 ** @return  0 if success,
 **         -1 if fail.  
 **/
int rm_regist_warp_image(const Registration *const self,
                         const unsigned char *const src,
						 unsigned char *const dst)
{
	int x, y, c;
	float rx, ry;
	int tlcx, tlcy;
	int lrcx, lrcy;
	float *citptr, *ritptr;
	unsigned char nval, sval;
	unsigned char neval, seval;
	unsigned char nwval, swval;
	int interp_val;
	enum {NCHANNELS = 3};
	unsigned char *src_udata;
	unsigned char *src_vdata;
	unsigned char *dst_udata;
	unsigned char *dst_vdata;
	int srcuv_width;
	int dstuv_width;
	int src_uvx, src_uvy;
	int dst_uvx, dst_uvy;
	
	assert(self);
	assert(src);
	assert(dst);
	
	memset(dst + self->base_width * self->base_height, 0x80, self->base_width * self->base_height >> 1);
	
	src_udata = (unsigned char *)src + self->unreg_width * self->unreg_height;
	src_vdata = (unsigned char *)src + self->unreg_width * self->unreg_height * 5 / 4;
	srcuv_width = self->unreg_width >> 1;
	
	dst_udata = dst + self->base_width * self->base_height;
	dst_vdata = dst + self->base_width * self->base_height * 5 / 4;
	dstuv_width = self->base_width >> 1;
	
	for (y = 0; y < self->base_height; y++) {
		citptr = self->col_inter_tab + y * self->base_width;
		ritptr = self->row_inter_tab + y * self->base_width;
		for (x = 0; x < self->base_width; x++) {
			rx = *(citptr + x);
			ry = *(ritptr + x);
			
			tlcx = (int)(rx);
			if (tlcx < 0 || tlcx > self->unreg_width - 1) {
				continue;
			}
			
			tlcy = (int)(ry);
			if (tlcy < 0 || tlcy > self->unreg_height - 1) {
				continue;
			}
			
			lrcx = tlcx + 1;
			if (lrcx < 0 || lrcx > self->unreg_width - 1) {
				continue;
			}
			
			lrcy = tlcy + 1;
			if (lrcy < 0 || lrcy > self->unreg_height - 1) {
				continue;
			}
#if 0			
			for (c = 0; c < NCHANNELS; c++) {
				nwval = src[tlcy * self->unreg_width * 3 + tlcx * 3 + c];
				swval = src[lrcy * self->unreg_width * 3 + tlcx * 3 + c];
				neval = src[tlcy * self->unreg_width * 3 + lrcx * 3 + c];
				seval = src[lrcy * self->unreg_width * 3 + lrcx * 3 + c];
				
				nval = (int)((rx - tlcx) * neval + (tlcx + 1 - rx) * nwval);
                sval = (int)((rx - tlcx) * seval + (tlcx + 1 - rx) * swval);
				
				interp_val = (int)((tlcy + 1 - ry) * nval + (ry - tlcy) * sval);
				if (interp_val < 0) {
					interp_val = 0;
				}
				
				if (interp_val > 255) {
					interp_val = 255;
				}
				
				dst[y * self->base_width * 3 + x * 3 + c] = interp_val;
			}
#else
			/* Y */
			nwval = src[tlcy * self->unreg_width + tlcx];
			swval = src[lrcy * self->unreg_width + tlcx];
			neval = src[tlcy * self->unreg_width + lrcx];
			seval = src[lrcy * self->unreg_width + lrcx];
			
			nval = (int)((rx - tlcx) * neval + (tlcx + 1 - rx) * nwval);
			sval = (int)((rx - tlcx) * seval + (tlcx + 1 - rx) * swval);
			
			interp_val = (int)((tlcy + 1 - ry) * nval + (ry - tlcy) * sval);
			if (interp_val < 0) {
				interp_val = 0;
			}
			
			if (interp_val > 255) {
				interp_val = 255;
			}
			
			dst[y * self->base_width + x] = interp_val;
			
			/* UV */
			src_uvx = tlcx >> 1;
			src_uvy = tlcy >> 1;
			dst_uvx = x >> 1;
			dst_uvy = y >> 1;
						
			if (0 == y % 2 && 0 == x % 2) {
				dst_udata[dst_uvy * dstuv_width + dst_uvx] = src_udata[src_uvy * srcuv_width + src_uvx];
				dst_vdata[dst_uvy * dstuv_width + dst_uvx] = src_vdata[src_uvy * srcuv_width + src_uvx];
			}
#endif
		}
	}
	
	return 0;
}

/** @brief Load matrix from file.
 ** @param filename matrix filename.
 ** @param tab interpolation table.
 ** @param rows columns of table.
 ** @param cols rows of table.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int load_interp_table(const char *filename,
                      float *const tab, int rows, int cols)
{
	FILE *fp;
	int x, y;
	
	assert(tab);
	
	if (_access(filename, 0)) {
		return -1;
	}
	
	if (!(fp = fopen(filename, "r"))) {
		return -1;
	}
	
	for (y = 0; y < rows; y++) {
		for (x = 0; x < cols; x++) {
			fscanf(fp, "%f ", &tab[y * cols + x]);
		}
	}
	
	fclose(fp);
	
	return 0;
}

/** @brief Calculate affine matrix.
 ** @param contrl_points control points.
 ** @param npoints number of control points.
 ** @param affine_matrix affine matrix.
 **/
void cal_affine_matrix(const int *const contrl_points, int npoints,
                       float *const affine_matrix)
{
	int i;
	int pairs;
	int x1, y1;
	int x2, y2;
	enum {PAIR_SIZE = 4};
	float abc_mat[12];
	float def_mat[12];
	int *cpptr;
	
	assert(contrl_points);
	assert(affine_matrix);
	
	pairs = npoints / 2;
	
	memset(abc_mat, 0, sizeof(abc_mat));
	memset(def_mat, 0, sizeof(def_mat));
	
	/* Construct augmented matrix with least square method. */
	for (i = 0; i < pairs; i++) {
		cpptr = (int *)(contrl_points + i * PAIR_SIZE);
		x1 = *cpptr;
		y1 = *(cpptr + 1);
		x2 = *(cpptr + 2);
		y2 = *(cpptr + 3);
		
		abc_mat[0] += x1 * x1;
		abc_mat[1] += x1 * y1;
		abc_mat[2] += x1;
		abc_mat[3] += x1 * x2;
		abc_mat[4] += x1 * y1;
		abc_mat[5] += y1 * y1;
		abc_mat[6] += y1;
		abc_mat[7] += x2 * y1;
		abc_mat[8] += x1;
		abc_mat[9] += y1;
		abc_mat[10] += 1;
		abc_mat[11] += x2;

		def_mat[0] += x1 * x1;
		def_mat[1] += x1 * y1;
		def_mat[2] += x1;
		def_mat[3] += x1 * y2;
		def_mat[4] += x1 * y1;
		def_mat[5] += y1 * y1;
		def_mat[6] += y1;
		def_mat[7] += y1 * y2;
		def_mat[8] += x1;
		def_mat[9] += y1;
		def_mat[10] += 1;
		def_mat[11] += y2;
	}
	
	{
		for (i = 0; i < 12; i++) {
			printf("%f ", abc_mat[i]);
		}
		printf("\n");
		for (i = 0; i < 12; i++) {
			printf("%f ", def_mat[i]);
		}
	}
	
	/* Solve linear equations with gaussian elimination. */
	ge_solver(abc_mat, 3);
	ge_solver(def_mat, 3);
	
	affine_matrix[0] = abc_mat[3];
	affine_matrix[1] = abc_mat[7];
	affine_matrix[2] = abc_mat[11];
	affine_matrix[3] = def_mat[3];
	affine_matrix[4] = def_mat[7];
	affine_matrix[5] = def_mat[11];
	
	{
		printf("\n");
		for (i = 0; i < 6; i++) {
			printf("%f ", affine_matrix[i]);
		}
	}
}

/** @brief Calculate interpolation table.
 ** @param affine_matrix affine matrix.
 ** @param base_width width of base image.
 ** @param base_height height of base image.
 ** @param unreg_width width of unregistered image.
 ** @param unreg_height height of unregistered image.
 ** @param row_inter_tab row interpolation table.
 ** @param col_inter_tab column interpolation table.
 **/
void cal_interp_table(const float *const affine_matrix,
                      int base_width, int base_height,
					  int unreg_width, int unreg_height,
					  float *const row_inter_tab,
					  float *const col_inter_tab)
{
	int x, y;
	float rx, ry;
	
	assert(affine_matrix);
	assert(row_inter_tab);
	assert(col_inter_tab);
	
	for (y = 0; y < base_height; y++) {
		for (x = 0; x < base_width; x++) {
			rx = affine_matrix[0] * x + affine_matrix[1] * y + affine_matrix[2];
			ry = affine_matrix[3] * x + affine_matrix[4] * y + affine_matrix[5];
			col_inter_tab[y * base_width + x] = rx;
			row_inter_tab[y * base_width + x] = ry;
		}
	}
}

/** @brief Save interpolation table.
 ** @param row_inter_tab row interpolation table.
 ** @param col_inter_tab column interpolation table.
 ** @param rows rows of interpolation table.
 ** @param cols columns of interpolation table.
 ** @param rtf row interpolation table filename.
 ** @param ctf column interpolation table filename.
 **/
void save_interp_table(const float *const tab,
					   int rows, int cols,
					   const char *filename)
{
	FILE *fp;
	int x, y;
	
	assert(tab);
		
	fp = fopen(filename, "w");
	assert(fp);
	
	for (y = 0; y < rows; y++) {
		for (x = 0; x < cols; x++) {
			fprintf(fp, "%f ", tab[y * cols + x]);
		}
		fputs("\n", fp);
	}
	
	fclose(fp);
}

/** @brief Swap two rows of matrix.
 ** @param mat augmented matrix.
 ** @param order coefficient matrix order.
 ** @param rowa row a.
 ** @param rowb row b.
 **/
void ge_swap_row(float *const mat, int order,
                 int rowa, int rowb)
{
	int x;
	int cols;
	float temp;
	float *linea;
	float *lineb;
	
	assert(mat);
	
	cols = order + 1;
	linea = mat + rowa * cols;
	lineb = mat + rowb * cols;
	
	for (x = rowa; x < cols; x++) {		
		temp = *(linea + x);
		*(linea + x) = *(lineb + x);
		*(lineb + x) = temp;
	}
}

/** @brief Select primary element of matrix.
 ** @param mat augmented matrix.
 ** @param order coefficient matrix order.
 **/ 
void ge_select_primary_element(float *const mat, int order)
{
	int cols;
	int x, y, x2;
	int primary_element_row;
	int diagonal_element_row;
	float primary_element_val, val;
	float diagonal_element_val;
	float k;
	
	assert(mat);
	
	cols = order + 1;
	
	for (x = 0; x < order; x++) {
		diagonal_element_row = x;
		primary_element_row = diagonal_element_row;
		primary_element_val = mat[primary_element_row * cols + x];
		
		/* Select primary element row. */
		for (y = diagonal_element_row; y < order; y++) {
			val = mat[y * cols + x];
			if (fabs(val) > fabs(primary_element_val)) {
				primary_element_row = y;
			}
		}
		
		/* Swap current row and primary element row if necessary. */
		if (primary_element_row != diagonal_element_row) {
			ge_swap_row(mat, order, diagonal_element_row, primary_element_row);
		}
		
		diagonal_element_val = mat[diagonal_element_row * cols + x];
		for (y = diagonal_element_row + 1; y < order; y++) {
			val = mat[y * cols + x];
			k = val / diagonal_element_val;
			for (x2 = x + 1; x2 < cols; x2++) {
				mat[y * cols + x2] -= k * mat[diagonal_element_row * cols + x2];
			}
		}
	}
}

/** @brief Solve homogeneous linear equations with gaussian elimination.
 ** @param mat augmented matrix.
 ** @param order coefficient matrix order.
 **/
void ge_solver(float *const mat, int order)
{
	int x, y;
	int cols;
	
	assert(mat);
	
	cols = order + 1;
	
	ge_select_primary_element(mat, order);
	
	for (y = order - 1; y >= 0; y--) {
		for (x = y + 1; x < order; x++) {
			mat[y * cols + order] -= mat[y * cols + x] * mat[x * cols + order];
		}
		
		mat[y * cols + order] /= mat[y * cols + y];
	}
}