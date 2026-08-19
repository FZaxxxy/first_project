/**
 * @file    joint.h
 * @brief   单关节运动接口
 * @note    第二阶段 2.1 运动控制层
 *
 * 在 servo（硬件驱动层）之上再封装一层：
 *   - 对目标角度做"关节限位"裁剪，防止超出机械行程；
 *   - 提供统一的 单关节绝对定位 / 平滑运动 接口。
 */
#ifndef __JOINT_H
#define __JOINT_H

#include "config.h"

/* ==================== 对外接口 ==================== */

/** 初始化（内部调用 Servo_Init） */
void Joint_Init(void);

/** 关节角度限位裁剪：超出则钳到 [min,max] */
float Joint_ClampAngle(uint8_t joint, float deg);

/** 单关节绝对定位（立即到位） */
void Joint_SetAngle(uint8_t joint, float deg);

/** 单关节平滑运动（duration_ms 内匀速到位） */
void Joint_MoveTo(uint8_t joint, float deg, float duration_ms);

/** 读取当前关节角度（度） */
float Joint_GetAngle(uint8_t joint);

/** 查询关节是否运动完成 */
uint8_t Joint_IsMoving(uint8_t joint);

#endif /* __JOINT_H */