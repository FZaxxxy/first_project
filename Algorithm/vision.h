/**
 * @file    vision.h
 * @brief   目标识别与坐标转换
 * @note    第三阶段 3.2 智能算法层
 *
 * 流程：OpenMV 视觉模块检测目标 -> 上报像素坐标
 *       -> Vision 模块按"针孔模型 + 相机安装参数"转换到
 *          机械臂基座坐标系（得到可执行的抓取点 x/y/z）。
 *
 * ★ 须自定义：相机安装位置/倾角/视场角等标定参数见 config.h。
 */
#ifndef __VISION_H
#define __VISION_H

#include "config.h"

/* ==================== 对外接口 ==================== */

/** 初始化视觉模块 */
void Vision_Init(void);

/** 像素坐标 -> 基座坐标系水平位置（x, y，单位 m）。
 *  以桌面平面 z=0 为投影面。 */
void Vision_PixelToBase(float px, float py, float *x, float *y);

/** 获取当前抓取目标（自动完成坐标转换 + 可靠性校验）
 *  @return 1=有有效目标 0=无目标/数据过期 */
uint8_t Vision_GetTarget(float *x, float *y, float *z);

#endif /* __VISION_H */