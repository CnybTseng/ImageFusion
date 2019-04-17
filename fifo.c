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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "fifo.h"

#ifndef is_power_of_2
#define is_power_of_2(x) ((x) != 0 && (((x) & ((x) - 1)) == 0))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

struct tagFifo
{
	char *buffer;				/**< ring buffer. */
	unsigned int size;			/**< ring buffer byte size. */
	unsigned int in;			/**< queue input. */
	unsigned int out;			/**< queue output. */
	pthread_mutex_t *mutex;		/**< mutual exclusion lock. */
};

/** @name some private functions
 ** @{ */
static unsigned int roundup_power_of_2(unsigned int a);
static unsigned int __fifo_len(const Fifo *self);
static unsigned int __fifo_put(Fifo *self, const char *buffer, unsigned int size);
static unsigned int __fifo_get(Fifo *self, char *buffer, unsigned int size);
/** @} */

/** @brief Create a new instance of ring buffer.
 ** @param size ring buffer byte size.
 ** @return the new instance.
 **/
Fifo *fifo_alloc(unsigned int size)
{
	char *buffer = NULL;
	pthread_mutex_t *mutex = NULL;
	Fifo *fifo = NULL;
	
	if (!is_power_of_2(size)) {
		size = roundup_power_of_2(size);
	}
	
	buffer = (char *)malloc(size);
	if (!buffer) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	mutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
	if (!mutex) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	if (0 != pthread_mutex_init(mutex, NULL)) {
		fprintf(stderr, "mutex init fail[%u:%s].\n", errno, strerror(errno));
		goto clean;
	}
	
	fifo = fifo_init(buffer, size, mutex);
	if (!fifo) {
		clean:
		if (buffer) {
			free(buffer);
			buffer = NULL;
		}
		if (mutex) {
			pthread_mutex_destroy(mutex);
			free(mutex);
			mutex = NULL;
		}
	}
	
	return fifo;
}

/** @brief Create ring buffer with given buffer and mutex.
 ** @param buffer allocated buffer.
 ** @param size allocated buffer size.
 ** @param mutex mutual exclusion lock.
 ** @return the new instance.
 **/
Fifo *fifo_init(char *buffer, unsigned int size, pthread_mutex_t *mutex)
{
	Fifo *self = NULL;
	
	assert(buffer);
	
	/*if (!is_power_of_2(size)) {
		fprintf(stderr, "fifo size must be power of 2.\n");
		return self;
	}*/
	
	self = (Fifo *)malloc(sizeof(Fifo));
	if (!self) {
		fprintf(stderr, "malloc fail[%u:%s].\n", errno, strerror(errno));
		return self;
	}
	
	memset(self, 0, sizeof(Fifo));
	self->buffer = buffer;
	self->size = size;
	self->in = 0;
	self->out = 0;
	self->mutex = mutex;
	
	return self;
}

/** @brief Delete ring buffer instance.
 ** @param self ring buffer instance.
 **/
void fifo_delete(Fifo *self)
{
	if (self) {
		if (self->buffer) {
			free(self->buffer);
			self->buffer = NULL;
		}
		if (self->mutex) {
			pthread_mutex_destroy(self->mutex);
			free(self->mutex);
			self->mutex = NULL;
		}
		free(self);
		self = NULL;
	}
}

/** @brief Get ring buffer filling bytes.
 ** @param self ring buffer instance.
 ** @return ring buffer filling bytes.
 **/
unsigned int fifo_len(const Fifo *self)
{
	unsigned int len;
	pthread_mutex_lock(self->mutex);
	len = __fifo_len(self);
	pthread_mutex_unlock(self->mutex);
	
	return len;
}

/** @brief Put data in ring buffer.
 ** @param self ring buffer instance.
 ** @param buffer data buffer.
 ** @param size data buffer size.
 ** @return writed data size.
 **/
unsigned int fifo_put(Fifo *self, const char *buffer, unsigned int size)
{
	unsigned int ret;
	pthread_mutex_lock(self->mutex);
	ret = __fifo_put(self, buffer, size);
	pthread_mutex_unlock(self->mutex);
	
	return ret;
}

/** @brief Get data from ring buffer.
 ** @param self ring buffer instance.
 ** @param buffer data buffer.
 ** @param size data buffer size.
 ** @return readed data size.
 **/
unsigned int fifo_get(Fifo *self, char *buffer, unsigned int size)
{
	unsigned int ret;
	pthread_mutex_lock(self->mutex);
	ret = __fifo_get(self, buffer, size);
	if (self->in == self->out) {
		self->in = self->out = 0;
	}
	
	pthread_mutex_unlock(self->mutex);
	
	return ret;
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

/** @brief Get ring buffer filling bytes.
 ** @param self ring buffer instance.
 ** @return ring buffer filling bytes.
 **/
unsigned int __fifo_len(const Fifo *self)
{
	return (self->in - self->out);
}

/** @brief Put data in ring buffer.
 ** @param self ring buffer instance.
 ** @param buffer data buffer.
 ** @param size data buffer size.
 ** @return writed data size.
 **/
unsigned int __fifo_put(Fifo *self, const char *buffer, unsigned int size)
{
	unsigned int len = 0;
	
	assert(self);
	assert(buffer);
	
	size = min(size, self->size - self->in + self->out);
	/* first put the data starting from self->in to buffer end */
	len = min(size, self->size - (self->in & (self->size - 1)));
	memcpy(self->buffer + (self->in & (self->size - 1)), buffer, len);
	/* then put the rest (if any) at the beginning of the buffer */
	memcpy(self->buffer, buffer + len, size - len);
	self->in += size;
	
	return size;
}

/** @brief Get data from ring buffer.
 ** @param self ring buffer instance.
 ** @param buffer data buffer.
 ** @param size data buffer size.
 ** @return readed data size.
 **/
unsigned int __fifo_get(Fifo *self, char *buffer, unsigned int size)
{
	unsigned int len = 0;
	
	assert(self);
	assert(buffer);
	
	size = min(size, self->in - self->out);
	/* first get the data from self->out until the end of the buffer */
	len = min(size, self->size - (self->out & (self->size - 1)));
	memcpy(buffer, self->buffer + (self->out & (self->size - 1)), len);
	/* then get the rest (if any) from the beginning of the buffer */
	memcpy(buffer + len, self->buffer, size - len);
	self->out += size;
	
	return size;
}