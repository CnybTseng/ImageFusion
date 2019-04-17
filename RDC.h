/*!
** \file RDC.h
** ===============================================================================
** Copyright (c) 2017, Chengdu ZLT Technology Co.,Ltd
**
** All Rights Reserved.
**
** module ....:
**
** Created By: chen xu bo
**
** Purpose:
**
** History:
**
** Programmer      mm/dd/yy Ver   Description
** --------------- -------- ----- ------------------------------
** chenxubo        01/09/18 00.00 Initial creation for Raw Data Converter.
**                                This component is used convert raw data to yuv data which can be
**                                handled by Hi3516
**
** ===============================================================================
*/

#ifndef _H_RDC
#define _H_RDC

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

//-----------------------------------------------------------------------------
//头文件

//-----------------------------------------------------------------------------
//宏定义


//-----------------------------------------------------------------------------
//枚举定义


//-----------------------------------------------------------------------------
//结构体

//-----------------------------------------------------------------------------
//全局变量定义

//-----------------------------------------------------------------------------
//静态变量定义


//-----------------------------------------------------------------------------
//函数声明

/**
 * [RDC_Init]
 * @param  enVideoFmt        
 *             The output frame format.
 *             ref typedef enum hiPIXEL_FORMAT_E
 *             22: PIXEL_FORMAT_YUV_SEMIPLANAR_422,
 *             23: PIXEL_FORMAT_YUV_SEMIPLANAR_420, 
 *             88: debug purpose, 
 *             the others value is invalid so far.
 *             
 * @param  enFrameResolution
 *             The frame resolution of raw data.
 *             ref typedef enum TStandardEx
 *             15: 384x288, 
 *             16: 640*480
 *                           
 * @return
 *             0: success
 *             -1: fail
 */
int RDC_Init(int enVideoFmt, int enFrameResolution);

/**
 * [RDC_SendRawData]
 *             Send one frame raw data to RDC. Non-blocking call, return immediately.
 *             
 * @param pu8Buf
 *             The buf pointer. The ownership of the buf is NOT transferred.
 *             
 * @param u32Len
 *             The buf len.
 */
void RDC_SendRawData(unsigned char * pu8Buf, unsigned int u32Len);


/**
 * [RDC_GetFrame]
 *             Get one video frame from RDC. The frame format should be defined when call RDC_Init().
 *             
 * @param  pu8Buf  
 *             The buf pointer. The ownership of the buf is NOT transferred.
 *             
 * @param  pu32Len 
 *             The buf len.
 *             
 * @return     
 *             0: success
 *             -1: fail
 */
int RDC_GetFrame(unsigned char * pu8Buf, unsigned int * pu32Len);


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif /* End of _H_RDC */