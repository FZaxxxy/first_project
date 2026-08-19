/**
 * @file    sensor.h
 * @brief   传感器接口（光电 / 夹爪限位 / 视觉输入）
 * @note    第三阶段 3.2 支持模块（BSP 层）
 *
 * 包含：
 *   - 光电传感器：检测物体是否在位（GPIO 电平）；
 *   - 夹爪限位开关：抓取到位确认（闭环反馈）；
 *   - 视觉数据缓存：OpenMV 等模块经串口上报的目标像素坐标。
 */
#ifndef __SENSOR_H
#define __SENSOR_H

#include "config.h"

/* ==================== 对外接口 ==================== */

/** 初始化传感器 GPIO 为输入 */
void Sensor_Init(void);

/** 读取光电传感器：1=检测到物体（★ 按实际电平极性调整） */
uint8_t Sensor_PhotoDetected(void);

/** 读取夹爪限位开关：1=夹爪已闭合到位（★ 按实际电平极性调整） */
uint8_t Sensor_GripClosed(void);

/** 视觉目标像素数据（由 OpenMV 串口解析后写入） */
typedef struct {
    uint8_t valid;      /* 1=当前帧检测到目标 */
    float   px;         /* 目标中心像素 x */
    float   py;         /* 目标中心像素 y */
    float   size;       /* 目标像素尺寸（直径/边长，用于距离估计） */
    uint32_t stamp;     /* 时间戳(HAL_GetTick)，判断数据是否过期 */
} Sensor_Vision_t;

/** 写入一帧视觉数据（由 OpenMV 指令解析调用） */
void Sensor_UpdateVision(uint8_t valid, float px, float py, float size);

/** 读取最近一帧视觉数据 */
void Sensor_GetVision(Sensor_Vision_t *v);

#endif /* __SENSOR_H */