/** @file vsg_init.h
 ** @brief Video stream grabber initialization
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

#ifndef _VSG_INIT_H_
#define _VSG_INIT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "vsg_stream.h"

void init_thread_attribute(video_pic_capture_context *context);
void init_ring_queue(video_pic_capture_context *context);

#ifdef __cplusplus
}
#endif

#endif


