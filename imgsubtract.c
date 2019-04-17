/** @file bkgsubtract.c - Implementation
 ** @brief Image subtraction
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef __WIN_SSE__	
#	include <smmintrin.h>
#endif

#ifdef __WIN_AVX__
#	include <immintrin.h>
#endif

#include "imgsubtract.h"

/** @name Some local functions.
 ** @{ */
static void img_subtract_kr_sse(const unsigned char *A, unsigned int width,
                                unsigned int height, const unsigned char *B,
					            unsigned char *C);
static void img_subtract_kr_avx(const unsigned char *A, unsigned int width,
                                unsigned int height, const unsigned char *B,
					            unsigned char *C);
static void img_subtract_kr_nsu(const unsigned char *A, unsigned int width,
                                unsigned int height, const unsigned char *B,
					            unsigned char *C);
static void img_subtract_sse(const unsigned char *A, unsigned int width,
                             unsigned int height, const unsigned char *B,
			                 short *C);
static void img_subtract_avx(const unsigned char *A, unsigned int width,
                             unsigned int height, const unsigned char *B,
			                 short *C);
static void img_subtract_nsu(const unsigned char *A, unsigned int width,
                             unsigned int height, const unsigned char *B,
			                 short *C);
/** @} */

/** @brief Subtract one image from another.
 **        keep gray range no changed.
 ** @param A one gray image.
 ** @param width image width.
 ** @param height image height.
 ** @param B another gray image.
 ** @param C difference image.
 **/
void img_subtract_kr(const unsigned char *A, unsigned int width,
                     unsigned int height, const unsigned char *B,
				     unsigned char *C)
{
	assert(A);
	assert(B);
	assert(C);
	
#ifdef __WIN_SSE__
	img_subtract_kr_sse(A, width, height, B, C);
#elif __WIN_AVX__
	img_subtract_kr_avx(A, width, height, B, C);
#else
	img_subtract_kr_nsu(A, width, height, B, C);
#endif
}

/** @brief Subtract one image from another.
 ** @param A one gray image.
 ** @param width image width.
 ** @param height image height.
 ** @param B another gray image.
 ** @param C difference image.
 **/
void img_subtract(const unsigned char *A, unsigned int width,
                  unsigned int height, const unsigned char *B,
			      short *C)
{
	assert(A);
	assert(B);
	assert(C);
	
#ifdef __WIN_SSE__
	img_subtract_sse(A, width, height, B, C);
#elif __WIN_AVX__
	img_subtract_avx(A, width, height, B, C);
#else
	img_subtract_nsu(A, width, height, B, C);
#endif
}

/** @brief Subtract one image from another with SSE.
 **        keep gray range no changed.
 ** @param A one gray image.
 ** @param width image width.
 ** @param height image height.
 ** @param B another gray image.
 ** @param C difference image.
 **/
#ifdef __WIN_SSE__
void img_subtract_kr_sse(const unsigned char *A, unsigned int width,
                         unsigned int height, const unsigned char *B,
					     unsigned char *C)
{
	unsigned int i;
	unsigned int npixels;
	const unsigned int ppl = 16;
	
	__m128i X;
	__m128i Y;
	__m128i Z;
	__m128i D;
	
	npixels = width * height;
	
	for (i = 0; i < npixels; i += ppl) {
		X = _mm_loadu_si128((__m128i *)(A + i));
		Y = _mm_loadu_si128((__m128i *)(B + i));
		Z = _mm_min_epu8(X, Y);
		D = _mm_sub_epi8(X, Z);
		_mm_storeu_si128((__m128i *)(C + i), D);
	}
}
#endif							 

/** @brief Subtract one image from another with AVX.
 **        keep gray range no changed.
 ** @param A one gray image.
 ** @param width image width.
 ** @param height image height.
 ** @param B another gray image.
 ** @param C difference image.
 **/
#ifdef __WIN_AVX__ 
void img_subtract_kr_avx(const unsigned char *A, unsigned int width,
                         unsigned int height, const unsigned char *B,
					     unsigned char *C)
{
	unsigned int i;
	unsigned int npixels;
	const unsigned int ppl = 32;
	
	__m256i X;
	__m256i Y;
	__m256i Z;
	__m256i D;
	
	npixels = width * height;
	
	for (i = 0; i < npixels; i += ppl) {
		X = _mm256_loadu_si256((__m256i *)(A + i));
		Y = _mm256_loadu_si256((__m256i *)(B + i));
		Z = _mm256_min_epu8(X, Y);
		D = _mm256_subs_epu8(X, Z);
		_mm256_storeu_si256((__m256i *)(C + i), D);
	}
}
#endif							 

/** @brief Subtract one image from another no speed up.
 **        keep gray range no changed.
 ** @param A one gray image.
 ** @param width image width.
 ** @param height image height.
 ** @param B another gray image.
 ** @param C difference image.
 **/							 
void img_subtract_kr_nsu(const unsigned char *A, unsigned int width,
                         unsigned int height, const unsigned char *B,
				         unsigned char *C)
{
	unsigned int i;
	unsigned int npixels;
	unsigned char val1;
	unsigned char val2;
	
	npixels = width * height;
	
	for (i = 0; i < npixels; i++) {
		val1 = A[i];
		val2 = B[i];
		if (val1 > val2) {
			C[i] = val1 - val2;
		} else {
			C[i] = 0;
		}
	}
}	

/** @brief Subtract one image from another with SSE.
 ** @param A one gray image.
 ** @param width image width.
 ** @param height image height.
 ** @param B another gray image.
 ** @param C difference image.
 **/
#ifdef __WIN_SSE__
void img_subtract_sse(const unsigned char *A, unsigned int width,
                      unsigned int height, const unsigned char *B,
			          short *C)
{
	unsigned int i;
	unsigned int j;
	unsigned int npixels;
	const unsigned int ppl = 16;
	const unsigned int pps = 8;
	
	__m128i Zero;
	__m128i X;
	__m128i XL;
	__m128i XH;
	__m128i Y;
	__m128i YL;
	__m128i YH;
	__m128i DL;
	__m128i DH;
	
	j = 0;
	npixels = width * height;
	Zero = _mm_setzero_si128();
	
	for (i = 0; i < npixels; i += ppl) {
		X = _mm_load_si128((__m128i *)(A + i));
		XL = _mm_unpacklo_epi8(X, Zero);
		XH = _mm_unpackhi_epi8(X, Zero);
		
		Y = _mm_load_si128((__m128i *)(B + i));
		YL = _mm_unpacklo_epi8(X, Zero);
		YH = _mm_unpackhi_epi8(X, Zero);
		
		DL = _mm_sub_epi16(XL, YL);
		DH = _mm_sub_epi16(XH, YH);
		
		_mm_store_si128((__m128i *)(C + j), DL);
		j += pps;
		_mm_store_si128((__m128i *)(C + j), DH);
		j += pps;
	}
}
#endif

/** @brief Subtract one image from another with AVX.
 ** @param A one gray image.
 ** @param width image width.
 ** @param height image height.
 ** @param B another gray image.
 ** @param C difference image.
 **/		
#ifdef __WIN_AVX__		
void img_subtract_avx(const unsigned char *A, unsigned int width,
                      unsigned int height, const unsigned char *B,
			          short *C)
{
	unsigned int i;
	unsigned int j;
	unsigned int npixels;
	const unsigned int ppl = 32;
	const unsigned int pps = 16;
	
	__m256i Zero;
	__m256i X;
	__m256i XL;
	__m256i XH;
	__m256i Y;
	__m256i YL;
	__m256i YH;
	__m256i DL;
	__m256i DH;
	
	j = 0;
	npixels = width * height;
	Zero = _mm256_setzero_si256();
	
	for (i = 0; i < npixels; i += ppl) {
		X = _mm256_load_si256((__m256i *)(A + i));
		XL = _mm256_unpacklo_epi8(X, Zero);
		XH = _mm256_unpackhi_epi8(X, Zero);
		
		Y = _mm256_load_si256((__m256i *)(B + i));
		YL = _mm256_unpacklo_epi8(Y, Zero);
		YH = _mm256_unpackhi_epi8(Y, Zero);
		
		DL = _mm256_sub_epi16(XL, YL);
		DH = _mm256_sub_epi16(XH, YH);
		
		_mm256_store_si256((__m256i *)(C + j), DL);
		j += pps;
		_mm256_store_si256((__m256i *)(C + j), DH);
		j += pps;
	}
}
#endif
	
/** @brief Subtract one image from another no speed up.
 ** @param A one gray image.
 ** @param width image width.
 ** @param height image height.
 ** @param B another gray image.
 ** @param C difference image.
 **/	
void img_subtract_nsu(const unsigned char *A, unsigned int width,
                      unsigned int height, const unsigned char *B,
			          short *C)	
{
	unsigned int i;
	unsigned int npixels;
	
	npixels = width * height;
	
	for (i = 0; i < npixels; i++) {
		C[i] = (short)A[i] - (short)B[i];
	}
}
							