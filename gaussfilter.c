/** @file gaussfilter.c - Implementation
 ** @brief IIR Gaussian blur filter.
 ** @author Zhiwei Zeng
 ** @date 2018.05.01
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
#include <tmmintrin.h>
#include <xmmintrin.h>
#include <smmintrin.h>
#include <immintrin.h>

#include "gaussfilter.h"

/** @typedef struct GFilterCoeff.
 ** @brief Gaussian filter coefficient.
 **/
typedef struct
{
	float a0;
	float a1;
	float a2;
	float a3;
	float b1;
	float b2;
	float cp;
	float cn;
}GFilterCoeff;

/** @name some private functions.
 ** @{ */
static void cal_gauss_coeff(float sigma,
                            GFilterCoeff *gfc);
static void gauss_filter_H_sse(unsigned char *line,
                               unsigned int width,
							   unsigned int height,
							   GFilterCoeff *gfc,
							   float *temp,
							   float *fline);
static void gauss_filter_V_sse(float *line,
                               unsigned int width,
							   unsigned int height,
							   GFilterCoeff *gfc,
							   float *temp,
							   unsigned char *fline);
static void gauss_filter_nsu(const unsigned char *image, unsigned int width,
                             unsigned int height, unsigned int ksize, float sigma,
							 unsigned char *gf_image);
/** @} */

/** @brief IIR Gaussian filter.
 ** @param image single channel 8bit image.
 ** @param width image width.
 ** @param height image height.
 ** @param sigma standard deviation.
 ** @param gf_image gaussian filtered image.
 **/ 
void gauss_filter(unsigned char *image,
                  unsigned int width,
				  unsigned int height,
				  float sigma,
				  unsigned char *gf_image)
{
#if 0
	GFilterCoeff gfc;
	unsigned int len = 0;
	unsigned int i = 0;
	float *ltr_buf = NULL;
	float *imf_buf = NULL;
	unsigned char *ptrr = NULL;
	float *ptrc = NULL;
	
	assert(image);
	assert(gf_image);
	
	len = ((width > height) ? width : height);
	len *= 4;

	ltr_buf = (float *)malloc(len * sizeof(float));
	if (!ltr_buf) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		return;
	}
	
	imf_buf = (float *)malloc(width * height * sizeof(float));
	if (!imf_buf) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		return;
	}
	
	cal_gauss_coeff(sigma, &gfc);
	
	ptrr = image;
	ptrc = imf_buf;
	
	/* horizontal filter. */
	for (i = 0; i < height; i += 4) {
		gauss_filter_H_sse(ptrr, width, height, &gfc, ltr_buf, ptrc);
		ptrr += (width << 2);
		ptrc += 4;;
	}

	ptrc = imf_buf;
	ptrr = gf_image;
	
	/* vertical filter. */
	for (i = 0; i < width; i += 4) {
		gauss_filter_V_sse(ptrc, height, width, &gfc, ltr_buf, ptrr);
		ptrc += (height << 2);
		ptrr += 4;
	}
	
	if (ltr_buf) {
		free(ltr_buf);
		ltr_buf = NULL;
	}
	
	if (imf_buf) {
		free(imf_buf);
		imf_buf = NULL;
	}
#else
	unsigned int ksize = 5;
	gauss_filter_nsu(image, width, height, ksize, sigma, gf_image);
#endif
}

/** @brief Calculate gaussian filter coefficient.
 ** @param sigma standard deviation of filter.
 ** @param gfc gaussian filter coefficient.
 **/
void cal_gauss_coeff(float sigma,
                     GFilterCoeff *gfc)
{
	float alpha;
	float lamda;
	float k;
	
	if (sigma < 0.5f) {
		sigma = 0.5f;
	}
	
	alpha = (float)exp(0.726 * 0.726) / sigma;
	lamda = (float)exp(-alpha);
	
	gfc->b1 = -2 * lamda;
	gfc->b2 = (float)exp(-2 * alpha);
	
	k = (1 - lamda) * (1 - lamda) / (1 + 2 * alpha * lamda - gfc->b2);
	
	gfc->a0 = k;
	gfc->a1 = k * (alpha - 1) * lamda;
	gfc->a2 = k * (alpha + 1) * lamda;
	gfc->a3 = -k * gfc->b2;
	
	gfc->cp = (gfc->a0 + gfc->a1) / (1 + gfc->b1 + gfc->b2);
	gfc->cn = (gfc->a2 + gfc->a3) / (1 + gfc->b1 + gfc->b2);
}

/** @brief Horizontal filter width SSE.
 ** @param line input image line. 
 ** @param width image width.
 ** @param height image height.
 ** @param gfc gaussian filter coeffes.
 ** @param ltr_buf left to right filter buffer.
 ** @param imf_buf intermediate filter buffer.
 **/
void gauss_filter_H_sse(unsigned char *line,
                        unsigned int width,
					    unsigned int height,
						GFilterCoeff *gfc,
						float *ltr_buf,
						float *imf_buf)
{
	__m128 A0;
	__m128 A1;
	__m128 A2;
	__m128 A3;
	__m128 B1;
	__m128 B2;
	__m128 CP;
	__m128 CN;
	
	__m128 X1;
	__m128 X1_1;
	__m128 X2;
	__m128 X2_1;
	__m128 X3;
	__m128 X3_1;
	__m128 X4;
	__m128 X4_1;
	__m128 XP;
	__m128 X_1P;
	__m128 XS;
	
	__m128 Y1;
	__m128 Y1_1;
	__m128 Y1_2;
	__m128 Y2;
	__m128 Y2_1;
	__m128 Y2_2;
	__m128 Y3;
	__m128 Y3_1;
	__m128 Y3_2;
	__m128 Y4;
	__m128 Y4_1;
	__m128 Y4_2;
	__m128 Y_1P;
	__m128 Y_2P;
	__m128 YS;
	
	__m128 T1;
	__m128 T2;
	__m128 T3;
	__m128 T4;
	
	unsigned int i = 0;
	unsigned char *ptrl = line;
	float *ptrt = ltr_buf;
	float *ptrf = imf_buf;
	
	A0 = _mm_load_ss(&gfc->a0);
	A0 = _mm_shuffle_ps(A0, A0, 0x00);
	A1 = _mm_load_ss(&gfc->a1);
	A1 = _mm_shuffle_ps(A1, A1, 0x00);
	A2 = _mm_load_ss(&gfc->a2);
	A2 = _mm_shuffle_ps(A2, A2, 0x00);
	A3 = _mm_load_ss(&gfc->a3);
	A3 = _mm_shuffle_ps(A3, A3, 0x00);
	B1 = _mm_load_ss(&gfc->b1);
	B1 = _mm_shuffle_ps(B1, B1, 0x00);
	B2 = _mm_load_ss(&gfc->b2);
	B2 = _mm_shuffle_ps(B2, B2, 0x00);
	CP = _mm_load_ss(&gfc->cp);
	CP = _mm_shuffle_ps(CP, CP, 0x00);
	CN = _mm_load_ss(&gfc->cn);
	CN = _mm_shuffle_ps(CN, CN, 0x00);
	
	X1_1 = _mm_cvtpu8_ps(\
	   _mm_movepi64_pi64(\
	     _mm_loadu_si128((__m128i *)ptrl)));
		 
	X2_1 = _mm_cvtpu8_ps(\
	   _mm_movepi64_pi64(\
	     _mm_loadu_si128((__m128i *)(ptrl + width))));
		 
	X3_1 = _mm_cvtpu8_ps(\
	   _mm_movepi64_pi64(\
	     _mm_loadu_si128((__m128i *)(ptrl + 2 * width))));
		 
	X4_1 = _mm_cvtpu8_ps(\
	   _mm_movepi64_pi64(\
	     _mm_loadu_si128((__m128i *)(ptrl + 3 * width))));
	 	 
	_MM_TRANSPOSE4_PS(X1_1, X2_1, X3_1, X4_1);	 
		 	 
	Y1_2 = _mm_mul_ps(X1_1, CP);
	Y1_1 = Y1_2;
		
	for (i = 0; i < width; i += 4) {
		X1 = _mm_cvtpu8_ps(\
	     _mm_movepi64_pi64(\
	       _mm_loadu_si128((__m128i *)ptrl)));

		X2 = _mm_cvtpu8_ps(\
	     _mm_movepi64_pi64(\
	       _mm_loadu_si128((__m128i *)(ptrl + width))));  

		X3 = _mm_cvtpu8_ps(\
	     _mm_movepi64_pi64(\
	       _mm_loadu_si128((__m128i *)(ptrl + 2 * width)))); 

		X4 = _mm_cvtpu8_ps(\
	     _mm_movepi64_pi64(\
	       _mm_loadu_si128((__m128i *)(ptrl + 3 * width))));
		   
		_MM_TRANSPOSE4_PS(X1, X2, X3, X4);
		   
		XP = _mm_mul_ps(X1, A0);
		X_1P = _mm_mul_ps(X1_1, A1);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y1_1, B1);
		Y_2P = _mm_mul_ps(Y1_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y1 = _mm_sub_ps(XS, YS);
		
		X2_1 = X1;
		Y2_2 = Y1_1;
		Y2_1 = Y1;
		 
		XP = _mm_mul_ps(X2, A0);
		X_1P = _mm_mul_ps(X2_1, A1);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y2_1, B1);
		Y_2P = _mm_mul_ps(Y2_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y2 = _mm_sub_ps(XS, YS);
		
		X3_1 = X2;
		Y3_2 = Y2_1;
		Y3_1 = Y2;
		
		XP = _mm_mul_ps(X3, A0);
		X_1P = _mm_mul_ps(X3_1, A1);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y3_1, B1);
		Y_2P = _mm_mul_ps(Y3_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y3 = _mm_sub_ps(XS, YS);
		
		X4_1 = X3;
		Y4_2 = Y3_1;
		Y4_1 = Y3;
		
		XP = _mm_mul_ps(X4, A0);
		X_1P = _mm_mul_ps(X4_1, A1);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y4_1, B1);
		Y_2P = _mm_mul_ps(Y4_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y4 = _mm_sub_ps(XS, YS);
		
		X1_1 = X4;
		Y1_2 = Y4_1;
		Y1_1 = Y4;
		
		_mm_storeu_ps((float *)(ptrt), Y1);
		_mm_storeu_ps((float *)(ptrt + 4), Y2);
		_mm_storeu_ps((float *)(ptrt + 8), Y3);
		_mm_storeu_ps((float *)(ptrt + 12), Y4);
		
		ptrl += 4;
		ptrt += 16;
	}
	
	ptrl -= 4;
	ptrt -= 16;
	ptrf += height * (width - 4);
	
	X4_1 = X1_1;	
	Y4_2 = _mm_mul_ps(X4_1, CN);
	Y4_1 = Y4_2;
	
	for (i = 0; i < width; i += 4) {
		X1 = _mm_cvtpu8_ps(\
	     _mm_movepi64_pi64(\
	       _mm_loadu_si128((__m128i *)(ptrl))));

		X2 = _mm_cvtpu8_ps(\
	     _mm_movepi64_pi64(\
	       _mm_loadu_si128((__m128i *)(ptrl + width)))); 

		X3 = _mm_cvtpu8_ps(\
	     _mm_movepi64_pi64(\
	       _mm_loadu_si128((__m128i *)(ptrl + 2 * width)))); 

		X4 = _mm_cvtpu8_ps(\
	     _mm_movepi64_pi64(\
	       _mm_loadu_si128((__m128i *)(ptrl + 3 * width))));
		   
		_MM_TRANSPOSE4_PS(X1, X2, X3, X4);
		
		T1 = _mm_loadu_ps(ptrt);
		T2 = _mm_loadu_ps(ptrt + 4);
		T3 = _mm_loadu_ps(ptrt + 8);
		T4 = _mm_loadu_ps(ptrt + 12);
		
		XP = _mm_mul_ps(X4, A2);
		X_1P = _mm_mul_ps(X4_1, A3);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y4_1, B1);
		Y_2P = _mm_mul_ps(Y4_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y4 = _mm_sub_ps(XS, YS); 
		
		X3_1 = X4;
		Y3_2 = Y4_1;
		Y3_1 = Y4; 
		 
		XP = _mm_mul_ps(X3, A2);
		X_1P = _mm_mul_ps(X3_1, A3);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y3_1, B1);
		Y_2P = _mm_mul_ps(Y3_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y3 = _mm_sub_ps(XS, YS); 
		
		X2_1 = X3;
		Y2_2 = Y3_1;
		Y2_1 = Y3; 
		 
		XP = _mm_mul_ps(X2, A2);
		X_1P = _mm_mul_ps(X2_1, A3);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y2_1, B1);
		Y_2P = _mm_mul_ps(Y2_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y2 = _mm_sub_ps(XS, YS); 
		 
		X1_1 = X2;
		Y1_2 = Y2_1;
		Y1_1 = Y2; 
		  
		XP = _mm_mul_ps(X1, A2);
		X_1P = _mm_mul_ps(X1_1, A3);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y1_1, B1);
		Y_2P = _mm_mul_ps(Y1_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y1 = _mm_sub_ps(XS, YS);

		X4_1 = X1;
		Y4_2 = Y1_1;
		Y4_1 = Y1;
		
		Y4 = _mm_add_ps(Y4, T4);
		Y3 = _mm_add_ps(Y3, T3);
		Y2 = _mm_add_ps(Y2, T2);
		Y1 = _mm_add_ps(Y1, T1);
		
		_mm_storeu_ps(ptrf, Y1);
		_mm_storeu_ps(ptrf + height, Y2);
		_mm_storeu_ps(ptrf + 2 * height, Y3);
		_mm_storeu_ps(ptrf + 3 * height, Y4);
		
		ptrl -= 4;
		ptrt -= 16;
		ptrf -= 4 * height;
	}
}

/** @brief Vertical filter width SSE.
 ** @param line intermediate image line.
 ** @param width image width.
 ** @param height image height.
 ** @param gfc gaussian filter coeffes.
 ** @param ltr_buf left to right filter buffer.
 ** @param gf_image gaussian filter image.
 **/
void gauss_filter_V_sse(float *line,
                        unsigned int width,
						unsigned int height,
						GFilterCoeff *gfc,
						float *ltr_buf,
						unsigned char *gf_image)
{
	__m128 A0;
	__m128 A1;
	__m128 A2;
	__m128 A3;
	__m128 B1;
	__m128 B2;
	__m128 CP;
	__m128 CN;
	
	__m128 X1;
	__m128 X1_1;
	__m128 X2;
	__m128 X2_1;
	__m128 X3;
	__m128 X3_1;
	__m128 X4;
	__m128 X4_1;
	__m128 XP;
	__m128 X_1P;
	__m128 XS;
	
	__m128 Y1;
	__m128 Y1_1;
	__m128 Y1_2;
	__m128 Y2;
	__m128 Y2_1;
	__m128 Y2_2;
	__m128 Y3;
	__m128 Y3_1;
	__m128 Y3_2;
	__m128 Y4;
	__m128 Y4_1;
	__m128 Y4_2;
	__m128 Y_1P;
	__m128 Y_2P;
	__m128 YS;
	__m128 T1;
	__m128 T2;
	__m128 T3;
	__m128 T4;
	__m128i Z;
	
	unsigned int i = 0;
	float *ptrl = line;
	float *ptrt = ltr_buf;
	unsigned char *ptrf = gf_image;
	
	A0 = _mm_load_ss(&gfc->a0);
	A0 = _mm_shuffle_ps(A0, A0, 0x00);
	A1 = _mm_load_ss(&gfc->a1);
	A1 = _mm_shuffle_ps(A1, A1, 0x00);
	A2 = _mm_load_ss(&gfc->a2);
	A2 = _mm_shuffle_ps(A2, A2, 0x00);
	A3 = _mm_load_ss(&gfc->a3);
	A3 = _mm_shuffle_ps(A3, A3, 0x00);
	B1 = _mm_load_ss(&gfc->b1);
	B1 = _mm_shuffle_ps(B1, B1, 0x00);
	B2 = _mm_load_ss(&gfc->b2);
	B2 = _mm_shuffle_ps(B2, B2, 0x00);
	CP = _mm_load_ss(&gfc->cp);
	CP = _mm_shuffle_ps(CP, CP, 0x00);
	CN = _mm_load_ss(&gfc->cn);
	CN = _mm_shuffle_ps(CN, CN, 0x00);
	
	X1_1 = _mm_loadu_ps(ptrl);
	X2_1 = _mm_loadu_ps(ptrl + width);
	X3_1 = _mm_loadu_ps(ptrl + 2 * width);
	X4_1 = _mm_loadu_ps(ptrl + 3 * width);
	
	_MM_TRANSPOSE4_PS(X1_1, X2_1, X3_1, X4_1);	
	
	Y1_2 = _mm_mul_ps(X1_1, CP);
	Y1_1 = Y1_2;
	
	for (i = 0; i < width; i += 4) {
		X1 = _mm_loadu_ps(ptrl);   
		X2 = _mm_loadu_ps(ptrl + width);  
		X3 = _mm_loadu_ps(ptrl + 2 * width);  
		X4 = _mm_loadu_ps(ptrl + 3 * width);
		
		_MM_TRANSPOSE4_PS(X1, X2, X3, X4);	
		
		XP = _mm_mul_ps(X1, A0);
		X_1P = _mm_mul_ps(X1_1, A1);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y1_1, B1);
		Y_2P = _mm_mul_ps(Y1_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y1 = _mm_sub_ps(XS, YS);
		
		X2_1 = X1;
		Y2_2 = Y1_1;
		Y2_1 = Y1;
		 
		XP = _mm_mul_ps(X2, A0);
		X_1P = _mm_mul_ps(X2_1, A1);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y2_1, B1);
		Y_2P = _mm_mul_ps(Y2_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y2 = _mm_sub_ps(XS, YS);
		
		X3_1 = X2;
		Y3_2 = Y2_1;
		Y3_1 = Y2;
		 
		XP = _mm_mul_ps(X3, A0);
		X_1P = _mm_mul_ps(X3_1, A1);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y3_1, B1);
		Y_2P = _mm_mul_ps(Y3_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y3 = _mm_sub_ps(XS, YS);
		
		X4_1 = X3;
		Y4_2 = Y3_1;
		Y4_1 = Y3;
		
		XP = _mm_mul_ps(X4, A0);
		X_1P = _mm_mul_ps(X4_1, A1);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y4_1, B1);
		Y_2P = _mm_mul_ps(Y4_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y4 = _mm_sub_ps(XS, YS);
		
		X1_1 = X4;
		Y1_2 = Y4_1;
		Y1_1 = Y4;
		
		_mm_storeu_ps(ptrt, Y1);
		_mm_storeu_ps(ptrt + 4, Y2);
		_mm_storeu_ps(ptrt + 8, Y3);
		_mm_storeu_ps(ptrt + 12, Y4);
		
		ptrl += 4;
		ptrt += 16;
	}
	
	ptrl -= 4;
	ptrt -= 16;
	ptrf += height * (width - 4);
	
	X4_1 = X1_1;	
	Y4_2 = _mm_mul_ps(X4_1, CN);
	Y4_1 = Y4_2;
	
	for (i = 0; i < width; i += 4) {
		X1 = _mm_loadu_ps(ptrl);   
		X2 = _mm_loadu_ps(ptrl + width);  
		X3 = _mm_loadu_ps(ptrl + 2 * width); 
		X4 = _mm_loadu_ps(ptrl + 3 * width); 
		
		_MM_TRANSPOSE4_PS(X1, X2, X3, X4);
		
		T1 = _mm_loadu_ps(ptrt);
		T2 = _mm_loadu_ps(ptrt + 4);
		T3 = _mm_loadu_ps(ptrt + 8);
		T4 = _mm_loadu_ps(ptrt + 12);
		
		XP = _mm_mul_ps(X4, A2);
		X_1P = _mm_mul_ps(X4_1, A3);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y4_1, B1);
		Y_2P = _mm_mul_ps(Y4_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y4 = _mm_sub_ps(XS, YS);
		
		X3_1 = X4;
		Y3_2 = Y4_1;
		Y3_1 = Y4;
		
		XP = _mm_mul_ps(X3, A2);
		X_1P = _mm_mul_ps(X3_1, A3);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y3_1, B1);
		Y_2P = _mm_mul_ps(Y3_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y3 = _mm_sub_ps(XS, YS);
		
		X2_1 = X3;
		Y2_2 = Y3_1;
		Y2_1 = Y3;
		
		XP = _mm_mul_ps(X2, A2);
		X_1P = _mm_mul_ps(X2_1, A3);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y2_1, B1);
		Y_2P = _mm_mul_ps(Y2_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y2 = _mm_sub_ps(XS, YS);
		
		X1_1 = X2;
		Y1_2 = Y2_1;
		Y1_1 = Y2;
		
		XP = _mm_mul_ps(X1, A2);
		X_1P = _mm_mul_ps(X1_1, A3);
		XS = _mm_add_ps(XP, X_1P);
		Y_1P = _mm_mul_ps(Y1_1, B1);
		Y_2P = _mm_mul_ps(Y1_2, B2);
		YS = _mm_add_ps(Y_1P, Y_2P);
		Y1 = _mm_sub_ps(XS, YS);
		
		X4_1 = X1;
		Y4_2 = Y1_1;
		Y4_1 = Y1;
		
		Y4 = _mm_add_ps(Y4, T4);
		Y3 = _mm_add_ps(Y3, T3);
		Y2 = _mm_add_ps(Y2, T2);
		Y1 = _mm_add_ps(Y1, T1);
		
		Z = _mm_cvtps_epi32(Y1);
		Z = _mm_packs_epi32(Z, Z);
		Z = _mm_packus_epi16(Z, Z);
		*((unsigned int *)ptrf) = _mm_cvtsi128_si32(Z);

		Z = _mm_cvtps_epi32(Y2);
		Z = _mm_packs_epi32(Z, Z);
		Z = _mm_packus_epi16(Z, Z);
		*((unsigned int *)(ptrf + height)) = _mm_cvtsi128_si32(Z);
		
		Z = _mm_cvtps_epi32(Y3);
		Z = _mm_packs_epi32(Z, Z);
		Z = _mm_packus_epi16(Z, Z);
		*((unsigned int *)(ptrf + 2 * height)) = _mm_cvtsi128_si32(Z);
		
		Z = _mm_cvtps_epi32(Y4);
		Z = _mm_packs_epi32(Z, Z);
		Z = _mm_packus_epi16(Z, Z);
		*((unsigned int *)(ptrf + 3 * height)) = _mm_cvtsi128_si32(Z);
		
		ptrl -= 4;
		ptrt -= 16;
		ptrf -= 4 * height;
	}
}

/** @brief Gaussian filter no speed up.
 ** @param image input image.
 ** @param width image width.
 ** @param height image height.
 ** @param gf_image gaussian filtered image.
 **/
void gauss_filter_nsu(const unsigned char *image, unsigned int width,
                      unsigned int height, unsigned int ksize, float sigma,
				      unsigned char *gf_image)
{
	unsigned int x, y;
	unsigned int i, kx, ky;
	unsigned int krad = ksize >> 1;
	const float pi = 3.141592653f;
	float kernel[1024];
	float dx, dy, kval;
	float sum = 0;
	
	for (y = 0; y < ksize; y++) {
		for (x = 0; x < ksize; x++) {
			dx = (float)x - krad;
			dy = (float)y - krad;
			kval = expf(-(dx * dx + dy * dy) / 2 / sigma / sigma);
			kval /= (sqrtf(2 * pi) * sigma);
			kernel[y * ksize + x] = kval;
			sum += kval;
		}
	}
	
	for (y = 0; y < ksize; y++) {
		for (x = 0; x < ksize; x++) {
			kernel[y * ksize + x] /= sum;
		}
	}
	
	for (y = krad; y < height - krad; y++) {
		for (x = krad; x < width - krad; x++) {
			i = 0;
			sum = 0;
			for (ky = y - krad; ky <= y + krad; ky++) {
				for (kx = x - krad; kx <= x + krad; kx++) {
					sum += image[ky * width + kx] * kernel[i];
				}
			}
			
			gf_image[y * width + x] = (unsigned int)(sum);
		}
	}
	
	for (y = 0; y < krad; y++) {
		memmove(gf_image + y * width, gf_image + krad * width, width * sizeof(unsigned char));
	}
	
	for (y = height - krad; y < height; y++) {
		memmove(gf_image + y * width, gf_image + (height - krad - 1) * width,
			width * sizeof(unsigned char));
	}
	
	for (y = 0; y < height; y++) {
		for (x = 0; x < krad; x++) {
			gf_image[y * width + x] = gf_image[y * width + krad];
		}
		
		for (x = width - krad; x < width; x++) {
			gf_image[y * width + x] = gf_image[y * width + width - krad - 1];
		}
	}
} 