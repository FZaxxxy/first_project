/**
 * @file    joint.c
 * @brief   单关节运动接口实现
 * @note    第二阶段 2.1 运动控制层
 */
#include "joint.h"
#include "servo.h"

/* 关节限位表：与 config.h 中 JOINT_LIMITS 对应 */
static const float g_joint_limits[JOINT_NUM][2] = JOINT_LIMITS;

/**
 * @brief  关节角度限位裁剪
 */
float Joint_ClampAngle(uint8_t joint, float deg)
{
    if (joint < 1 || joint > JOINT_NUM) {
        return 0.0f;
    }
    if (deg < g_joint_limits[joint - 1][0]) {
        return g_joint_limits[joint - 1][0];
    }
    if (deg > g_joint_limits[joint - 1][1]) {
        return g_joint_limits[joint - 1][1];
    }
    return deg;
}

void Joint_Init(void)
{
    Servo_Init();
}

void Joint_SetAngle(uint8_t joint, float deg)
{
    Servo_SetAngle(joint, Joint_ClampAngle(joint, deg));
}

void Joint_MoveTo(uint8_t joint, float deg, float duration_ms)
{
    Servo_SmoothMove(joint, Joint_ClampAngle(joint, deg), duration_ms);
}

float Joint_GetAngle(uint8_t joint)
{
    return Servo_GetAngle(joint);
}

uint8_t Joint_IsMoving(uint8_t joint)
{
    return Servo_IsMoving(joint);
}