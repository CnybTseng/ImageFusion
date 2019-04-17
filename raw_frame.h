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
 * 创建一个新的原始帧接收器.
 * @return 原始帧接收器指针.
 */
__declspec(dllexport) void *frame_receiver_new();

/**
 * 初始化原始帧接收器.
 * @param[in] handle 原始帧接收器指针.
 * @param[in] port 绑定端口.
 * @return True如果成功, 否则返回false.
 */
__declspec(dllexport) bool frame_receiver_init(void *const handle, uint16_t port);

/**
 * 从原始帧接收器提取数据.
 * @param[in] handle 原始帧接收器指针.
 * @param[in] data 原始数据指针.
 * @param[out] width 原始帧宽度.
 * @param[out] height 原始帧高度.
 * @return True如果成功, 否则返回false.
 */
__declspec(dllexport) bool frame_receiver_get(void *const handle, uint16_t *const data, int32_t *const width, int32_t *const height);

/**
 * 从原始帧接收器提取数据.
 * @param[in] handle 原始帧接收器指针.
 * @param[in] data 原始数据指针.
 * @param[out] width 原始帧宽度.
 * @param[out] height 原始帧高度.
 * @param[out] vtemp 探测器温度.
 * @return True如果成功, 否则返回false.
 */
__declspec(dllexport) bool frame_receiver_get(void *const handle, uint16_t *const data, int32_t *const width, int32_t *const height, double *vtemp);

/**
 * 停止原始帧接收器.
 * @param[in] handle 原始帧接收器指针.
 * @return void.
 */
__declspec(dllexport) void frame_receiver_stop(void *const handle);

/**
 * 释放原始帧接收器资源.
 * @param[in] handle 原始帧接收器指针.
 * @return void.
 */
__declspec(dllexport) void frame_receiver_free(void *const handle);

#endif