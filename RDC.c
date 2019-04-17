/** @file RDC.c
 ** @brief Raw data convert to YUV format - Implementation
 ** @author Zhiwei Zeng
 ** @date 2018.04.08
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the thermal camera toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h> 
#include <assert.h>

#ifdef __WIN_SSE__	
#	include <smmintrin.h>
#endif

#ifdef __ARM_NEON__ 
#	include <arm_neon.h>
#endif

#include "RDC.h"

#define DEBUG																(0)
#define FUNCTION_TEST														(0)
#define FRAME_RECOMBINED													(0)
#define NUMBER_OF_GRAYLEVELS												(0x3FFF + 1)
#define MAXIMUM_NUMBER_OF_PIXELS											(0x4B000)
#define MAXIMUM_BYTES_OF_YUV422												(MAXIMUM_NUMBER_OF_PIXELS * 2)
#define MAXIMUM_BYTES_OF_YUV420												(MAXIMUM_NUMBER_OF_PIXELS * 3 / 2)
#define MAXIMUM_BYTES_OF_RGB												(MAXIMUM_NUMBER_OF_PIXELS * 3)
#define UV_FILLED_VALUE														(0x80)

#define Min(a, b) (a < b ? a : b)
#define Max(a, b) (a > b ? a : b)

#if FUNCTION_TEST
#	include "timer.h"
#endif

/**
 * \typedef enum hiPIXEL_FORMAT_E
 * \brief Enumerate for the pixel format.
 */
typedef enum
{
	PIXEL_FORMAT_YUV_SEMIPLANAR_422 = 22,
	PIXEL_FORMAT_YUV_SEMIPLANAR_420,
	PIXEL_FORMAT_RGB,
	PIXEL_FORMAT_RGBA,
	PIXEL_FORMAT_YUV_DEBUG = 88
}hiPIXEL_FORMAT_E;

/**
 * \typedef enum TStandardEx
 * \brief Enumerate for the frame resolution.
 */
typedef enum
{
	FRAME_RESOLUTION_OF_384 = 15,
	FRAME_RESOLUTION_OF_640
}TStandardEx;

/**
 * \typedef enum GrayRange
 * \brief Enumerate for the gray range.
 */
typedef enum
{
	BLACK = 0,
	WHITE = 255
}GrayRange;

/**
 * \struct RDC
 * \brief Data structure for the RDC.
 */
struct RDC
{
	hiPIXEL_FORMAT_E videoFmt;								// Video format
	TStandardEx frameResolution;							// Frame resolution ID
	int width;												// Frame width
	int height;												// Frame height
	unsigned int cutThresh;									// Minimum threshold of histogram
	int nTilesX;											// Number of tiles in X direction
	int nTilesY;											// Number of tiles in Y direction
	int nBins;												// Number of bins of histogram
	float clipLimit;										// Histogram clip limit
	unsigned int rawDataLen;
	unsigned short *rawData;
	unsigned int outputDataLen;
	unsigned long clipLevel;								// Histogram clip level
	unsigned short map[NUMBER_OF_GRAYLEVELS];				// Rearrange map table
	unsigned char stretchMap[NUMBER_OF_GRAYLEVELS];			// Stretch map table
	unsigned long histogram[NUMBER_OF_GRAYLEVELS];
	unsigned long rearHist[NUMBER_OF_GRAYLEVELS];			// Rearranged histogram
	unsigned short recombData[MAXIMUM_NUMBER_OF_PIXELS];	// Recombined raw frame
	unsigned char claheData[MAXIMUM_NUMBER_OF_PIXELS];		// Stretched image
	unsigned char yuv422Data[MAXIMUM_BYTES_OF_YUV422];
	unsigned char yuv420Data[MAXIMUM_BYTES_OF_YUV420];
	unsigned char yuvDebugData[MAXIMUM_BYTES_OF_RGB];
};

// Define raw data formate converter.
static struct RDC dataConverter;

/**
 * Recombine raw frame.
 * @param src The source frame.
 * @param len Length of the source frame.
 * @param dst The destination frame.
 * @return
 */
static void RecombineRawFrame(unsigned char *src, unsigned int len, unsigned short *dst);

/**
 * Calculate histogram of image.
 * @param img Image for processing.
 * @param width Image width.
 * @param height Image height.
 * @param hist Histogram of image.
 * @param nBins Number of bins.
 * @return
 */
static void CalHist(unsigned short *img, int width, int height, unsigned long *hist, int nBins);

/**
 * Rearrange histogram of image.
 * @param hist Histogram of image.
 * @param nBins Number of bins of histogram.
 *                [0x3FFF + 1]
 * @param thresh Threshold for bin values.
 *                [4]
 * @param rearHist Rearranged histogram.
 * @param nValidBins Number of valid bins of histogram.
 * @param nValidPixs Total number of valid bins value.
 * @param map Map table from original to rearranged value.
 * @return
 */
static void RearrangeHist(unsigned long *hist, int nBins, unsigned int thresh, unsigned long *rearHist,
                          int *nValidBins, unsigned long *nValidPixs, unsigned short *map);

/**
 * Clip histogram of image.
 * @param hist Histogram of image. You shold call RearrangeHist before.
 * @param nBins Number of valid bins of histogram.
 * @param clipLevel Clip level.
 * @return
 */ 
static void ClipHist(unsigned long *hist, int nBins, unsigned long clipLevel);				   

/**
 * Stretch the histogram of image.
 * @param hist Histogram of image.
 * @param nBins Number of bins of histogram.
 * @param min Minimum of stretched image.
 *                [0]
 * @param max Maximum of stretched image.
 *                [255]
 * @param nPixels Number of pixels of image.
 * @param map Stretch map.
 * @return
 */
static void StretchHist(unsigned long *hist, int nBins, unsigned short min, unsigned short max,
                        unsigned long nPixels, unsigned char *map);
						
/**
 * Contrast limited adaptive histogram equalization.
 * @param src Source image.
 * @param width Source image width.
 * @param height Source image height.
 * @param nTilesX Number of rectangular contextual regions in X direction.
 *                  [1]
 * @param nTilesY Number of rectangular contextual regions in Y direction.
 *                  [1]
 * @param nBins Number of histogram bins used to build a contrast enhancing transformation.
 *                  [0x3FFF + 1]
 * @param clipLimit Contrast enhancement limit.
 *                  [0,1]
 * @param dst Destination image with the same resolution as the source image.
 * @return
 *               0: success
 *              -1: fail
 */
static int CLAHE(unsigned short *src, int width, int height, int nTilesX,
                 int nTilesY, int nBins, float clipLimit, unsigned char *dst);

/**
 * Convert unsigned char grayscale image to YUV422 image.
 * @param src Source image.
 * @param width Source image width.
 * @param height Source image height.
 * @param dst Destination image.
 * @return
 *                 0: success
 *                -1: fail
 */ 
static int U8C1ConvertToYUV422(unsigned char *src, int width, int height, unsigned char *dst);				 
				 
/**
 * Convert unsigned char grayscale image to YUV420 image.
 * @param src Source image.
 * @param width Source image width.
 * @param height Source image height.
 * @param dst Destination image.
 * @return
 *                 0: success
 *                -1: fail
 */ 
static int U8C1ConvertToYUV420(unsigned char *src, int width, int height, unsigned char *dst);

/**
 * Convert unsigned char grayscale image to RGB image.
 * @param src Source image.
 * @param width Source image width.
 * @param height Source image height.
 * @param dst Destination image.
 * @return
 *                 0: success
 *                -1: fail
 */ 
static int U8C1ConvertToRGB(unsigned char *src, int width, int height, unsigned char *dst);

/**
 * Convert unsigned char grayscale image to RGBA image.
 * @param src Source image.
 * @param width Source image width.
 * @param height Source image height.
 * @param dst Destination image.
 * @return
 *                 0: success
 *                -1: fail
 */
static int U8C1ConvertToRGBA(unsigned char *src, int width, int height, unsigned char *dst);

/**
 * Sets the eight unsigned 16-bit integer values to s.
 * @param s Unsigned short integer.
 * @return Eight unsigned 16-bit integer.
 */
#ifdef __WIN_SSE__
static __m128i _mm_set1_epu16(unsigned short s)
{
  __m128i m = _mm_cvtsi32_si128(s);
  m = _mm_shufflelo_epi16(m, _MM_SHUFFLE(0, 0, 0, 0));
  m = _mm_unpacklo_epi64(m, m);
  return m;
}
#endif

/**
 * Save histogram as file.
 * @param hist Histogram of image.
 * @param nBins Number of bins of histogram.
 * @param filename File name
 * @return
 */
static void SaveHistogram(unsigned long *hist, int nBins, char *filename)
{
	FILE *fp = fopen(filename, "w");
	assert(fp);
	
	int i;
	for (i = 0; i < nBins; i++) {
		fprintf(fp, "%lu\n", hist[i]);
	}
	
	fclose(fp);
}

/**
 * Save stretch table as file.
 * @param Stretch table.
 * @param size Size of stretch table.
 * @param filename File name
 * @return
 */
static void SaveStretchTab(unsigned char *map, int size, char *filename)
{
	FILE *fp = fopen(filename, "w");
	assert(fp);
	
	int i;
	for (i = 0; i < size; i++) {
		fprintf(fp, "%u\n", map[i]);
	}
	
	fclose(fp);
}

/**
 * Read YUV data from file.
 * @param dst Destination buffer.
 * @param len Data length.
 * @return
 *                0: success
 *               -1: fail
 */
static int ReadYUVFromFile(unsigned char *dst, unsigned int *len)
{
	const char filename[] = "yuv.dat";
	
	FILE *fp = fopen(filename, "rb");
	if (NULL == fp) {
		perror("fopen");
		return -1;
	}
	
	fread(dst, sizeof(unsigned char), dataConverter.outputDataLen, fp);
	fclose(fp);
	
	struct stat statbuf;  
    stat(filename, &statbuf); 
	*len = statbuf.st_size;
	
	return 0;
}

// -------------------------------------------------------------------------
// Init Raw Data Converter.
// -------------------------------------------------------------------------
int RDC_Init(int enVideoFmt, int enFrameResolution)
{	
	dataConverter.frameResolution = enFrameResolution;
	if (FRAME_RESOLUTION_OF_384 == enFrameResolution) {
		dataConverter.width = 384;
		dataConverter.height = 288;
	} else if (FRAME_RESOLUTION_OF_640 == enFrameResolution) {
		dataConverter.width = 640;
		dataConverter.height = 480;
	} else {
		return -1;
	}
	
	dataConverter.videoFmt = enVideoFmt;
	if (PIXEL_FORMAT_YUV_SEMIPLANAR_422 == enVideoFmt) {
		dataConverter.outputDataLen = dataConverter.width * dataConverter.height * 2;
	} else if (PIXEL_FORMAT_YUV_SEMIPLANAR_420 == enVideoFmt) {
		dataConverter.outputDataLen = dataConverter.width * dataConverter.height * 3 / 2;
	} else if (PIXEL_FORMAT_RGB == enVideoFmt) {
		dataConverter.outputDataLen = dataConverter.width * dataConverter.height * 3;
	} else if (PIXEL_FORMAT_RGBA == enVideoFmt) {
		dataConverter.outputDataLen = dataConverter.width * dataConverter.height * 4;
	} else if (PIXEL_FORMAT_YUV_DEBUG == enVideoFmt) {
		dataConverter.outputDataLen = dataConverter.width * dataConverter.height * 3;
	} else {
		return -1;
	}
	
	dataConverter.cutThresh = 4;
	dataConverter.nTilesX = 1;
	dataConverter.nTilesY = 1;
	dataConverter.nBins = 0x3FFF + 1;
	dataConverter.clipLimit = 1;
	dataConverter.rawData = NULL;
		
	return 0;
}

// -------------------------------------------------------------------------
// Send one frame raw data to RDC.
// -------------------------------------------------------------------------
void RDC_SendRawData(unsigned char * pu8Buf, unsigned int u32Len)
{
#if FRAME_RECOMBINED
	dataConverter.rawData = (unsigned short *)pu8Buf;
#else
#if FUNCTION_TEST
	StartTimer();
#endif
	RecombineRawFrame(pu8Buf, u32Len, dataConverter.recombData);
#if FUNCTION_TEST
	StopTimer("RecombineRawFrame");
#endif
	dataConverter.rawData = dataConverter.recombData;
#endif
	dataConverter.rawDataLen = u32Len;
}

// -------------------------------------------------------------------------
// Recombine raw frame.
// -------------------------------------------------------------------------
void RecombineRawFrame(unsigned char *src, unsigned int len, unsigned short *dst)
{
	assert(src);
	assert(dst);
	
	enum {BYTES_PER_PIXEL = 2};
	int nPixels = len / 2;
	int i;
#ifdef __ARM_NEON__
{
#if 0
	for (i = 0; i < nPixels; i++) {
		unsigned short low8bits = 0x00FF & src[i * BYTES_PER_PIXEL];
		unsigned short hig8bits = src[i * BYTES_PER_PIXEL + 1] & 0x7F;
		dst[i] = (hig8bits << 8) + low8bits;
	}
#else
	const int bytesPerLoad = 16;	
	uint8x16_t low_bits_mask = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
		0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
	uint8x16_t hig_bits_mask = {0x00, 0x7F, 0x00, 0x7F, 0x00, 0x7F, 0x00, 0x7F,
		0x00, 0x7F, 0x00, 0x7F, 0x00, 0x7F, 0x00, 0x7F};
	
	for (i = 0; i < len; i += bytesPerLoad) {
		unsigned char *pSrc = src + i;
		uint8x16_t src_data = vld1q_u8(pSrc);
		
		uint8x16_t left_shift_data = vshlq_n_u8(vandq_u8(src_data, hig_bits_mask), 1);
		uint16x8_t high_part_data = vreinterpretq_u16_u8(left_shift_data);
		
		uint8x16_t right_shift_data = vshrq_n_u8(vandq_u8(src_data, low_bits_mask), 1);
		uint16x8_t low_part_data = vreinterpretq_u16_u8(right_shift_data);
		
		uint16x8_t dst_data = vaddq_u16(high_part_data, low_part_data);
		
		unsigned short *pDst = dst + (i >> 1);
		vst1q_u16(pDst, dst_data);			
	}
#endif
}
#else
{
	for (i = 0; i < nPixels; i++) {
		unsigned short low8bits = 0x00FF & src[i * BYTES_PER_PIXEL];
		unsigned short hig8bits = src[i * BYTES_PER_PIXEL + 1] & 0x7F;
		dst[i] = (hig8bits << 8) + low8bits;
	}
}
#endif
}

// -------------------------------------------------------------------------
// Calculate histogram of image.
// -------------------------------------------------------------------------
void CalHist(unsigned short *img, int width, int height, unsigned long *hist, int nBins)
{
	assert(img);
	assert(hist);
	
	memset(hist, 0, nBins * sizeof(unsigned long));
	
	const int nPixels = width * height;
	int i, j = 0;
#ifdef __WIN_SSE__
{
	/*const int pixsPerLoad = 8;	
	for (i = 0; i < nPixels; i += pixsPerLoad) {
		const unsigned short *pData = img + i;
		__m128i data = _mm_loadu_si128((__m128i *)pData);
		
		unsigned short *ptr = (unsigned short *)&data;	
		for (j = 0; j < pixsPerLoad; j++) {
			hist[ptr[j]]++;
		}
	}*/
	for (i = 0; i < nPixels; i++) {
		hist[img[i]]++;
	}
}
#elif defined(__ARM_NEON__)
{	
	const int pixsPerLoad = 8;
	for (i = 0; i < nPixels; i += pixsPerLoad) {
		const unsigned short *pData = img + i;
		uint16x8_t data = vld1q_u16(pData);
		
		for (j = 0; j < pixsPerLoad; j++) {
			hist[data[j]]++;
		}
	}
}
#else
{
	for (i = 0; i < nPixels; i++) {
		hist[img[i]]++;
	}
}
#endif
}

// -------------------------------------------------------------------------
// Rearrange histogram of image.
// -------------------------------------------------------------------------
void RearrangeHist(unsigned long *hist, int nBins, unsigned int thresh, unsigned long *rearHist,
                   int *nValidBins, unsigned long *nValidPixs, unsigned short *map)
{
	assert(hist);
	assert(rearHist);
	assert(map);
	
	memset(rearHist, 0, nBins * sizeof(unsigned long));
	
	int nvbs = 0;
	unsigned long nvps = 0;
	int maxValidLevel = 0;
	int i;
	
	for (i = 0; i < nBins; i++) {
		if (hist[i] < thresh) {
			map[i] = nvbs;
			continue;
		}
		
		nvbs++;
		rearHist[nvbs - 1] = hist[i];
		map[i] = nvbs - 1;
		nvps += hist[i];
		maxValidLevel = i;
	}
	
	for (i = maxValidLevel + 1; i < nBins; i++) {
		map[i] = nvbs - 1;
	}
	
	*nValidBins = nvbs;
	*nValidPixs = nvps;
}

// -------------------------------------------------------------------------
// Clip histogram of image.
// -------------------------------------------------------------------------
void ClipHist(unsigned long *hist, int nBins, unsigned long clipLevel)
{
	assert(hist);
	
	int i, j;
	unsigned long nClipeds = 0;
	for (i = 0; i < nBins; i++) {
		long excess = hist[i] - clipLevel;
		if (excess > 0) {
			nClipeds += excess;
		}
	}
	
	unsigned long nRedists = nClipeds / nBins;
	unsigned long upper = clipLevel - nRedists;
	
	for (i = 0; i < nBins; i++) {
		if (hist[i] > clipLevel) {
			hist[i] = clipLevel;
		} else {
			if (hist[i] > upper) {
				nClipeds -= clipLevel - hist[i];
				hist[i] = clipLevel;
			} else {
				nClipeds -= nRedists;
				hist[i] += nRedists;
			}
		}
	}
	
	unsigned long nPrevClipeds;
	
	do {
		nPrevClipeds = nClipeds;
		for (i = 0; i < nBins && nClipeds; i++) {
			unsigned long step = nClipeds / nBins;
			step = Max(step, 1);
			for (j = i; j < nBins && nClipeds; j += step) {
				if (hist[j] < clipLevel) {
					nClipeds--;
					hist[j]++;
				}
			}
		}
	} while (nClipeds && nClipeds < nPrevClipeds);
}

// -------------------------------------------------------------------------
// Stretch the histogram of image.
// -------------------------------------------------------------------------
void StretchHist(unsigned long *hist, int nBins, unsigned short min, unsigned short max,
                 unsigned long nPixels, unsigned char *map)
{
	assert(hist);
	assert(map);
	
	unsigned long accum = 0;
	float scale = ((float)max - min) / nPixels;
	int i;
	
	for (i = 0; i < nBins; i++) {
		accum += hist[i];
		unsigned char val = (unsigned char)(min + scale * accum);
		map[i] = Min(val, max);
	}
}

// -------------------------------------------------------------------------
// Contrast limited adaptive histogram equalization.
// -------------------------------------------------------------------------
int CLAHE(unsigned short *src, int width, int height, int nTilesX,
          int nTilesY, int nBins, float clipLimit, unsigned char *dst)
{
	if (NULL == src || NULL == dst) {
		return -1;
	}

#if FUNCTION_TEST	
	StartTimer();
#endif
	CalHist(src, width, height, dataConverter.histogram, dataConverter.nBins);
#if FUNCTION_TEST
	StopTimer("CalHist");
#endif

#if DEBUG
	SaveHistogram(dataConverter.histogram, dataConverter.nBins, "hist.txt");
#endif
	
	int nValidBins = 0;
	unsigned long nValidPixs = 0;
	
#if FUNCTION_TEST	
	StartTimer();
#endif	
	RearrangeHist(dataConverter.histogram, dataConverter.nBins, dataConverter.cutThresh, dataConverter.rearHist,
		&nValidBins, &nValidPixs, dataConverter.map);
#if FUNCTION_TEST
	StopTimer("RearrangeHist");
#endif

#if DEBUG
	SaveHistogram(dataConverter.rearHist, nValidBins, "rhist.txt");
#endif
	
	dataConverter.clipLevel = (unsigned long)(dataConverter.clipLimit * dataConverter.width *
		dataConverter.height / nValidBins);
		
#if FUNCTION_TEST	
	StartTimer();
#endif		
	ClipHist(dataConverter.rearHist, nValidBins, dataConverter.clipLevel);
#if FUNCTION_TEST
	StopTimer("ClipHist");
#endif

#if DEBUG	
	SaveHistogram(dataConverter.rearHist, nValidBins, "chist.txt");
#endif
	
	unsigned long nPixels = dataConverter.width * dataConverter.height;
	
#if FUNCTION_TEST	
	StartTimer();
#endif
	StretchHist(dataConverter.rearHist, nValidBins, BLACK, WHITE, nPixels, dataConverter.stretchMap);
#if FUNCTION_TEST
	StopTimer("StretchHist");
#endif

#if DEBUG	
	SaveStretchTab(dataConverter.stretchMap, nValidBins, "map.txt");
#endif

#if FUNCTION_TEST	
	StartTimer();
#endif
	unsigned long i;
#ifdef __WIN_SSE__
{
	for (i = 0; i < nPixels; i++) {
		dataConverter.claheData[i] = dataConverter.stretchMap[dataConverter.map[src[i]]];
	}
}
#elif defined(__ARM_NEON__)
{
	int j;
	const int pixsPerLoad = 8;
	
	for (i = 0; i < nPixels; i += pixsPerLoad) {
		const unsigned short *pSrc = src + i;
		uint16x8_t src_data = vld1q_u16(pSrc);
		uint8x8_t dst_data;
		
		for (j = 0; j < pixsPerLoad; j++) {
			dst_data[j] = dataConverter.stretchMap[dataConverter.map[src_data[j]]];
		}
		
		unsigned char *pDst = dataConverter.claheData + i;
		vst1_u8(pDst, dst_data);
	}
}
#else
{
	for (i = 0; i < nPixels; i++) {
		dataConverter.claheData[i] = dataConverter.stretchMap[dataConverter.map[src[i]]];
	}
}
#endif	
#if FUNCTION_TEST
	StopTimer("stretchMap");
#endif
	
	return 0;
}

// -------------------------------------------------------------------------
// Convert unsigned char grayscale image to YUV422 image.
// -------------------------------------------------------------------------
int U8C1ConvertToYUV422(unsigned char *src, int width, int height, unsigned char *dst)
{
	if (NULL == src || NULL == dst) {
		return -1;
	}
	
	unsigned char *yData = dst;
	memmove(yData, src, width * height * sizeof(unsigned char));
	
	unsigned char *uvData = yData + width * height;
	const int nUVs = width * height;
	int i;
	
	for (i = 0; i < nUVs; i++) {
		uvData[i] = UV_FILLED_VALUE;
	}
	
	return 0;
}

// -------------------------------------------------------------------------
// Convert unsigned char grayscale image to YUV420 image.
// -------------------------------------------------------------------------
int U8C1ConvertToYUV420(unsigned char *src, int width, int height, unsigned char *dst)
{
	if (NULL == src || NULL == dst) {
		return -1;
	}
	
	unsigned char *yData = dst;
	memmove(yData, src, width * height * sizeof(unsigned char));
	
	unsigned char *uvData = yData + width * height;
	const int nUVs = width * (height / 2);
	int i;
	
	for (i = 0; i < nUVs; i++) {
		uvData[i] = UV_FILLED_VALUE;
	}
	
	return 0;
}

// -------------------------------------------------------------------------
// Convert unsigned char grayscale image to RGB image.
// -------------------------------------------------------------------------
int U8C1ConvertToRGB(unsigned char *src, int width, int height, unsigned char *dst)
{
	if (NULL == src || NULL == dst) {
		return -1;
	}
	
	enum {BYTES_PER_PIXEL = 3};
	int nPixels = width * height;
	int i;
	
	for (i = 0; i < nPixels; i++) {
		dst[BYTES_PER_PIXEL * i] = src[i];
		dst[BYTES_PER_PIXEL * i + 1] = src[i];
		dst[BYTES_PER_PIXEL * i + 2] = src[i];
	}
	
	return 0;
}

// -------------------------------------------------------------------------
// Convert unsigned char grayscale image to RGBA image.
// -------------------------------------------------------------------------
int U8C1ConvertToRGBA(unsigned char *src, int width, int height, unsigned char *dst)
{
	if (NULL == src || NULL == dst) {
		return -1;
	}
	
	enum {BYTES_PER_PIXEL = 4};
	int nPixels = width * height;
	int i;
	
	for (i = 0; i < nPixels; i++) {
		dst[BYTES_PER_PIXEL * i] = src[i];
		dst[BYTES_PER_PIXEL * i + 1] = src[i];
		dst[BYTES_PER_PIXEL * i + 2] = src[i];
		dst[BYTES_PER_PIXEL * i + 3] = 0;
	}
	
	return 0;
}

// -------------------------------------------------------------------------
// Get one video frame from RDC.
// -------------------------------------------------------------------------
int RDC_GetFrame(unsigned char * pu8Buf, unsigned int * pu32Len)
{
	if (NULL == pu8Buf) {
		return -1;
	}
	
	if (CLAHE(dataConverter.rawData, dataConverter.width, dataConverter.height, dataConverter.nTilesX,
		dataConverter.nTilesY, dataConverter.nBins, dataConverter.clipLimit, dataConverter.claheData)) {
		return -1;
	}
	
	if (PIXEL_FORMAT_YUV_SEMIPLANAR_422 == dataConverter.videoFmt) {
		if (U8C1ConvertToYUV422(dataConverter.claheData, dataConverter.width, dataConverter.height, pu8Buf)) {
			return -1;
		}
		
		*pu32Len = dataConverter.outputDataLen;
	} else if (PIXEL_FORMAT_YUV_SEMIPLANAR_420 == dataConverter.videoFmt) {
		if (U8C1ConvertToYUV420(dataConverter.claheData, dataConverter.width, dataConverter.height, pu8Buf)) {
			return -1;
		}
		
		*pu32Len = dataConverter.outputDataLen;
	} else if (PIXEL_FORMAT_RGB == dataConverter.videoFmt) {
		if (U8C1ConvertToRGB(dataConverter.claheData, dataConverter.width, dataConverter.height, pu8Buf)) {
			return -1;
		}
		
		*pu32Len = dataConverter.outputDataLen;
	} else if (PIXEL_FORMAT_RGBA == dataConverter.videoFmt) {
		if (U8C1ConvertToRGBA(dataConverter.claheData, dataConverter.width, dataConverter.height, pu8Buf)) {
			return -1;
		}
		
		*pu32Len = dataConverter.outputDataLen;
	} else if (PIXEL_FORMAT_YUV_DEBUG == dataConverter.videoFmt) {
		if (ReadYUVFromFile(pu8Buf, pu32Len)) {
			return -1;
		}
	} else {
		return -1;
	}
	
	return 0;
}