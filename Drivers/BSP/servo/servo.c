/**
 * @file    servo.c
 * @brief   舵机角度映射与平滑控制实现
 * @note    第一阶段 1.3 硬件驱动层
 */
#include "servo.h"
#include <math.h>

/* ------------------------------------------------------------------
 * 舵机状态（每关节一份）
 * ------------------------------------------------------------------ */
typedef struct {
    float cur;          /* 当前角度（度） */
    float target;       /* 目标角度（度） */
    float step;         /* 每节拍步进角度（度） */
    uint8_t moving;     /* 1=平滑运动中 */
} Servo_Joint_t;

static PCA9685_Cascade_t g_cas;
static Servo_Joint_t     g_servo[JOINT_NUM];
static float             g_gripper_ms = 0.0f;

/**
 * @brief  角度(度) -> 脉宽(ms) 线性映射
 *
 * 关节角 0 度 对应舵机中位 SERVO_MID_DEG（默认 90 度）。
 * pulse = MIN_MS + (MID_DEG + deg) / RANGE_DEG * (MAX_MS - MIN_MS)
 */
float Servo_AngleToPulseMs(float joint_deg)
{
    float servo_deg = SERVO_MID_DEG + joint_deg;
    float pulse;
    pulse = SERVO_MIN_PULSE_MS
          + (servo_deg / SERVO_RANGE_DEG) * (SERVO_MAX_PULSE_MS - SERVO_MIN_PULSE_MS);
    /* 物理限幅：防止越界 */
    if (pulse < SERVO_MIN_PULSE_MS) pulse = SERVO_MIN_PULSE_MS;
    if (pulse > SERVO_MAX_PULSE_MS) pulse = SERVO_MAX_PULSE_MS;
    return pulse;
}

/**
 * @brief  初始化：级联板初始化 + 关节归零 + 夹爪张开
 */
void Servo_Init(void)
{
    uint8_t i;

    /* 初始化两块级联板 */
    PCA9685_Cascade_Init(&g_cas);

    /* 各关节初始状态：0 度 */
    for (i = 0; i < JOINT_NUM; i++) {
        g_servo[i].cur     = 0.0f;
        g_servo[i].target  = 0.0f;
        g_servo[i].step    = 0.0f;
        g_servo[i].moving  = 0;
        PCA9685_Cascade_SetJointPulseMs(&g_cas, i + 1,
                                        Servo_AngleToPulseMs(0.0f));
    }

    /* 夹爪默认张开 */
    g_gripper_ms = GRIPPER_OPEN_MS;
    PCA9685_Cascade_SetPulseMs(&g_cas, GRIPPER_BOARD, GRIPPER_CHANNEL,
                               g_gripper_ms);
}

/**
 * @brief  立即设置某关节角度（不做平滑）
 */
void Servo_SetAngle(uint8_t joint, float deg)
{
    if (joint < 1 || joint > JOINT_NUM) return;

    g_servo[joint - 1].cur    = deg;
    g_servo[joint - 1].target = deg;
    g_servo[joint - 1].moving = 0;

    PCA9685_Cascade_SetJointPulseMs(&g_cas, joint, Servo_AngleToPulseMs(deg));
}

/**
 * @brief  平滑运动设置
 *
 * @param duration_ms  期望运动时长(ms)。若为 0 则视为立即到位。
 *
 * 内部按控制节拍 MOTION_TICK_MS 计算"每节拍步进角度"。
 * 还以 SERVO_MAX_SPEED_DPS 限幅，防止单关节超速。
 */
void Servo_SmoothMove(uint8_t joint, float target_deg, float duration_ms)
{
    float delta, ticks, step;
    float max_step;   /* 由角速度上限折算的每节拍最大步进 */

    if (joint < 1 || joint > JOINT_NUM) return;

    g_servo[joint - 1].target = target_deg;
    delta = target_deg - g_servo[joint - 1].cur;

    if (fabsf(delta) < 0.01f || duration_ms <= 0.0f) {
        /* 已到位或要求立即到位 */
        g_servo[joint - 1].step   = 0.0f;
        g_servo[joint - 1].moving = 0;
        g_servo[joint - 1].cur    = target_deg;
        PCA9685_Cascade_SetJointPulseMs(&g_cas, joint,
                                        Servo_AngleToPulseMs(target_deg));
        return;
    }

    /* 每节拍步进 = 总变化 / 节拍数 */
    ticks = duration_ms / (float)MOTION_TICK_MS;
    if (ticks < 1.0f) ticks = 1.0f;
    step = delta / ticks;

    /* 角速度限幅保护 */
    max_step = SERVO_MAX_SPEED_DPS * (float)MOTION_TICK_MS / 1000.0f;
    if (fabsf(step) > max_step) {
        step = (step > 0.0f) ? max_step : -max_step;
    }

    g_servo[joint - 1].step   = step;
    g_servo[joint - 1].moving = 1;
}

/**
 * @brief  夹爪控制：open=1 张开，open=0 闭合
 */
void Servo_SetGripper(uint8_t open)
{
    g_gripper_ms = open ? GRIPPER_OPEN_MS : GRIPPER_CLOSE_MS;
    PCA9685_Cascade_SetPulseMs(&g_cas, GRIPPER_BOARD, GRIPPER_CHANNEL,
                               g_gripper_ms);
}

/**
 * @brief  每节拍推进（放入定时器中断）
 */
void Servo_Update(void)
{
    uint8_t i;

    for (i = 0; i < JOINT_NUM; i++) {
        if (!g_servo[i].moving) continue;

        g_servo[i].cur += g_servo[i].step;

        /* 越过目标则吸附到目标并停止 */
        if ((g_servo[i].step > 0.0f && g_servo[i].cur >= g_servo[i].target) ||
            (g_servo[i].step < 0.0f && g_servo[i].cur <= g_servo[i].target)) {
            g_servo[i].cur    = g_servo[i].target;
            g_servo[i].moving = 0;
        }

        /* 刷新该关节 PWM 输出 */
        PCA9685_Cascade_SetJointPulseMs(&g_cas, i + 1,
                                        Servo_AngleToPulseMs(g_servo[i].cur));
    }
}

float Servo_GetAngle(uint8_t joint)
{
    if (joint < 1 || joint > JOINT_NUM) return 0.0f;
    return g_servo[joint - 1].cur;
}

uint8_t Servo_IsMoving(uint8_t joint)
{
    if (joint < 1 || joint > JOINT_NUM) return 0;
    return g_servo[joint - 1].moving;
}