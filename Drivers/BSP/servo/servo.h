/**
 * @file    servo.h
 * @brief   舵机角度映射与平滑控制
 * @note    第一阶段 1.3 硬件驱动层
 *
 * 功能：
 *   1. 角度(度) -> 脉宽(ms) -> PCA9685 计数值 的线性映射；
 *   2. 平滑控制：按设定时长，每个控制节拍(TIM 中断)递增角度，
 *      避免舵机瞬间跳变造成冲击/堵转。
 *
 * 使用方法：
 *   - 初始化：Servo_Init();
 *   - 定时器中周期调用：Servo_Update();  (周期 = MOTION_TICK_MS)
 *   - 运动：Servo_SmoothMove(joint, 目标角度, 时长ms);
 */
#ifndef __SERVO_H
#define __SERVO_H

#include "config.h"
#include "pca9685_cascade.h"

/* ==================== 对外接口 ==================== */

/** 初始化：级联板初始化 + 所有关节归零 + 夹爪张开 */
void Servo_Init(void);

/** 角度(度) -> 脉宽(ms)（线性映射，可单独测试用） */
float Servo_AngleToPulseMs(float joint_deg);

/** 立即设置某关节角度（不做平滑，直接输出） */
void Servo_SetAngle(uint8_t joint, float deg);

/** 平滑运动：在 duration_ms 内从当前角度匀速运动到目标角度 */
void Servo_SmoothMove(uint8_t joint, float target_deg, float duration_ms);

/** 夹爪控制：open=1 张开，open=0 闭合 */
void Servo_SetGripper(uint8_t open);

/** 每节拍调用一次（放入定时器中断）：
 *  推进各关节角度一步并刷新 PCA9685 输出 */
void Servo_Update(void);

/** 查询某关节当前角度（度） */
float Servo_GetAngle(uint8_t joint);

/** 查询某关节是否还在平滑运动中 */
uint8_t Servo_IsMoving(uint8_t joint);

#endif /* __SERVO_H */