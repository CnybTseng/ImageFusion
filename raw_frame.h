#ifndef _RAW_FRAME_H_
#define _RAW_FRAME_H_

#if defined( _WIN32) || defined(_WIN64)
#if _MSC_VER < 1600
typedef char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
#else
#include <cstdint>
#endif
#else
#include <cstdint>
#endif

/**
 * ����һ���µ�ԭʼ֡������.
 * @return ԭʼ֡������ָ��.
 */
__declspec(dllexport) void *frame_receiver_new();

/**
 * ��ʼ��ԭʼ֡������.
 * @param[in] handle ԭʼ֡������ָ��.
 * @param[in] port �󶨶˿�.
 * @return True����ɹ�, ���򷵻�false.
 */
__declspec(dllexport) bool frame_receiver_init(void *const handle, uint16_t port);

/**
 * ��ԭʼ֡��������ȡ����.
 * @param[in] handle ԭʼ֡������ָ��.
 * @param[in] data ԭʼ����ָ��.
 * @param[out] width ԭʼ֡���.
 * @param[out] height ԭʼ֡�߶�.
 * @return True����ɹ�, ���򷵻�false.
 */
__declspec(dllexport) bool frame_receiver_get(void *const handle, uint16_t *const data, int32_t *const width, int32_t *const height);

/**
 * ��ԭʼ֡��������ȡ����.
 * @param[in] handle ԭʼ֡������ָ��.
 * @param[in] data ԭʼ����ָ��.
 * @param[out] width ԭʼ֡���.
 * @param[out] height ԭʼ֡�߶�.
 * @param[out] vtemp ̽�����¶�.
 * @return True����ɹ�, ���򷵻�false.
 */
__declspec(dllexport) bool frame_receiver_get(void *const handle, uint16_t *const data, int32_t *const width, int32_t *const height, double *vtemp);

/**
 * ֹͣԭʼ֡������.
 * @param[in] handle ԭʼ֡������ָ��.
 * @return void.
 */
__declspec(dllexport) void frame_receiver_stop(void *const handle);

/**
 * �ͷ�ԭʼ֡��������Դ.
 * @param[in] handle ԭʼ֡������ָ��.
 * @return void.
 */
__declspec(dllexport) void frame_receiver_free(void *const handle);

#endif