/** @file imgmul.c - Implementation
 ** @brief Image add
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
#include <omp.h>

#ifdef __WIN_SSE__	
#	include <smmintrin.h>
#endif

#ifdef __WIN_AVX__
#	include <immintrin.h>
#endif

static void img_mul_s_kr_sse(const unsigned char *A, unsigned int width,
                             unsigned int height, float k, unsigned char *B);
static void img_mul_s_kr_avx(const unsigned char *A, unsigned int width,
                             unsigned int height, float k, unsigned char *B);
static void img_mul_s_kr_nsu(const unsigned char *A, unsigned int width,
                             unsigned int height, float k, unsigned char *B);

void img_mul_s_kr(const unsigned char *A, unsigned int width,
                  unsigned int height, float k, unsigned char *B)
{
	assert(A);
	assert(B);
	
#ifdef __WIN_SSE__
	img_mul_s_kr_sse(A, width, height, k, B);
#elif __WIN_AVX__
	img_mul_s_kr_avx(A, width, height, k, B);
#else
	img_mul_s_kr_nsu(A, width, height, k, B);
#endif
}

void img_mul_s_kr_sse(const unsigned char *A, unsigned int width,
                      unsigned int height, float k, unsigned char *B)
{
	img_mul_s_kr_nsu(A, width, height, k, B);
}					  

#ifdef __WIN_AVX__					  
void img_mul_s_kr_avx(const unsigned char *A, unsigned int width,
                      unsigned int height, float k, unsigned char *B)
{
	unsigned int i;
	unsigned int npixels;
	const unsigned int ppl = 16;

	__m128i Zero;
	__m128i X;
	__m128i XL;
	__m128i XH;
	__m256i XL_u32;
	__m256i XH_u32;
	__m256  XL_f32;
	__m256  XH_f32;
	__m256  YL_f32;
	__m256  YH_f32;
	__m256i YL_u32;
	__m256i YH_u32;
	__m128i YLL_u32;
	__m128i YLH_u32;
	__m128i YHL_u32;
	__m128i YHH_u32;
	__m128i YLL_u8;
	__m128i YLH_u8;
	__m128i YHL_u8;
	__m128i YHH_u8;
	__m128i Y;
	
	__m128i SHUFFLE_MASKYC_LL = {0x00, 0x04, 0x08, 0x0C,
	                             0x80, 0x80, 0x80, 0x80,
						         0x80, 0x80, 0x80, 0x80,
						         0x80, 0x80, 0x80, 0x80};
	
	__m128i SHUFFLE_MASKYC_LH = {0x80, 0x80, 0x80, 0x80,
	                             0x00, 0x04, 0x08, 0x0C,
						         0x80, 0x80, 0x80, 0x80,
						         0x80, 0x80, 0x80, 0x80};
	
	__m128i SHUFFLE_MASKYC_HL = {0x80, 0x80, 0x80, 0x80,
	                             0x80, 0x80, 0x80, 0x80,
						         0x00, 0x04, 0x08, 0x0C,
						         0x80, 0x80, 0x80, 0x80};
	
	__m128i SHUFFLE_MASKYC_HH = {0x80, 0x80, 0x80, 0x80,
	                             0x80, 0x80, 0x80, 0x80,
						         0x80, 0x80, 0x80, 0x80,
						         0x00, 0x04, 0x08, 0x0C};
	
	npixels = width * height;
	Zero = _mm_setzero_si128();
	
	for (i = 0; i < npixels; i += ppl) {
		X = _mm_loadu_si128((__m128i *)(A + i));				/* u8x16 */
		
		XL = _mm_unpacklo_epi8(X, Zero);						/* u16x8 */
		XH = _mm_unpackhi_epi8(X, Zero);						/* u16x8 */
		
		XL_u32 = _mm256_cvtepu16_epi32(XL);						/* u32x8 */
		XH_u32 = _mm256_cvtepu16_epi32(XH);						/* u32x8 */
		
		XL_f32 = _mm256_cvtepi32_ps(XL_u32);					/* f32x8 */
		XH_f32 = _mm256_cvtepi32_ps(XH_u32);					/* f32x8 */
		
		YL_f32 = _mm256_mul_ps(XL_f32, _mm256_broadcast_ss(&k));
		YH_f32 = _mm256_mul_ps(XH_f32, _mm256_broadcast_ss(&k));
		
		YL_u32 = _mm256_cvtps_epi32(YL_f32);					/* u32x8 */
		YH_u32 = _mm256_cvtps_epi32(YH_f32);					/* u32x8 */
		
		YLL_u32 = _mm256_extractf128_si256(YL_u32, 0x00);		/* u32x4 */
		YLH_u32 = _mm256_extractf128_si256(YL_u32, 0x01);		/* u32x4 */
		YHL_u32 = _mm256_extractf128_si256(YH_u32, 0x00);		/* u32x4 */
		YHH_u32 = _mm256_extractf128_si256(YH_u32, 0x01);		/* u32x4 */
		
		YLL_u8 = _mm_shuffle_epi8(YLL_u32, SHUFFLE_MASKYC_LL);	/* u8x4 */
		YLH_u8 = _mm_shuffle_epi8(YLH_u32, SHUFFLE_MASKYC_LH);	/* u8x4 */
		YHL_u8 = _mm_shuffle_epi8(YHL_u32, SHUFFLE_MASKYC_HL);	/* u8x4 */
		YHH_u8 = _mm_shuffle_epi8(YHH_u32, SHUFFLE_MASKYC_HH);	/* u8x4 */
		
		Y = _mm_or_si128(_mm_or_si128(YLL_u8, YLH_u8),_mm_or_si128(YHL_u8, YHH_u8));
		
		_mm_storeu_si128((__m128i *)(B + i), Y);
	}
}
#endif					  
					  
void img_mul_s_kr_nsu(const unsigned char *A, unsigned int width,
                      unsigned int height, float k, unsigned char *B)
{
	unsigned int i;
	unsigned int npixels;

	npixels = width * height;
	
	for (i = 0; i < npixels; i++) {
		B[i] = (unsigned char)(k * A[i]);
	}
}					  