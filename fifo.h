/** @file fifo.h
 ** @brief Ring buffer
 ** @author Zhiwei Zeng
 ** @date 2018.04.13
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _FIFO_H_
#define _FIFO_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <pthread.h>

/** @typedef Fifo
 ** @brief ring buffer
 **/
struct tagFifo;
typedef struct tagFifo Fifo;

/** @name Allocate and destroy
 ** @{ */
Fifo *fifo_alloc(unsigned int size);
Fifo *fifo_init(char *buffer, unsigned int size, pthread_mutex_t *mutex);
void fifo_delete(Fifo *self);
/** @} */

/** @name FIFO operation
 ** @{ */
unsigned int fifo_len(const Fifo *self);
unsigned int fifo_put(Fifo *self, const char *buffer, unsigned int size);
unsigned int fifo_get(Fifo *self, char *buffer, unsigned int size);
/** @} */

#ifdef __cplusplus
}
#endif

#endif