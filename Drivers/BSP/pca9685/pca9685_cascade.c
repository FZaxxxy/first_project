/**
 * @file    pca9685_cascade.c
 * @brief   PCA9685 级联（双块）驱动实现
 * @note    第一阶段 1.2 硬件驱动层
 *
 * 接线说明：
 *   - 两片 PCA9685 的 SDA/SCL/VCC/GND 并联到 STM32 的同一组 I2C。
 *   - 板 1 地址引脚 A0~A5 全部接地 -> 0x40；
 *   - 板 2 的 A0 接高 -> 0x41；其余接地。
 *   （地址配置见 config.h）
 */
#include "pca9685_cascade.h"
#include <string.h>

static PCA9685_Cascade_t g_cascade;   /* 全局级联实例（模块内静态） */

/* ------------------------------------------------------------------
 * 关节 -> (板, 通道) 映射
 * 规则见 config.h：SERVO_JOINT_BOARD / SERVO_JOINT_CH。
 * 修改接线时只需改 config.h，本文件无需改动。
 * ------------------------------------------------------------------ */
static uint8_t joint_to_board(uint8_t joint)
{
    if (joint < 1 || joint > JOINT_NUM) return 0;
    return SERVO_JOINT_BOARD(joint);
}

static uint8_t joint_to_channel(uint8_t joint)
{
    if (joint < 1 || joint > JOINT_NUM) return 0;
    return SERVO_JOINT_CH(joint);
}

/**
 * @brief  初始化两块级联板
 */
void PCA9685_Cascade_Init(PCA9685_Cascade_t *cas)
{
    if (cas == 0) cas = &g_cascade;

    cas->count = 0;

    /* 板 1 */
    PCA9685_Init(&cas->boards[0], PCA9685_HI2C, PCA9685_BOARD1_ADDR);
    if (cas->boards[0].init_ok) {
        PCA9685_SetPWMFreq(&cas->boards[0], PCA9685_PWM_FREQ_HZ);
        cas->count++;
    }

    /* 板 2 */
    PCA9685_Init(&cas->boards[1], PCA9685_HI2C, PCA9685_BOARD2_ADDR);
    if (cas->boards[1].init_ok) {
        PCA9685_SetPWMFreq(&cas->boards[1], PCA9685_PWM_FREQ_HZ);
        cas->count++;
    }
}

/**
 * @brief  设置指定板、指定通道的脉宽
 */
void PCA9685_Cascade_SetPulseMs(PCA9685_Cascade_t *cas,
                                PCA9685_BoardId_t board,
                                uint8_t ch, float pulse_ms)
{
    if (cas == 0) cas = &g_cascade;
    if (board < BOARD_1 || board > BOARD_2) return;

    PCA9685_SetPulseMs(&cas->boards[board - 1], ch, pulse_ms);
}

/**
 * @brief  按关节号设置脉宽（自动路由到正确的板与通道）
 */
void PCA9685_Cascade_SetJointPulseMs(PCA9685_Cascade_t *cas,
                                     uint8_t joint, float pulse_ms)
{
    uint8_t board, ch;

    if (cas == 0) cas = &g_cascade;
    if (joint < 1 || joint > JOINT_NUM) return;

    board = joint_to_board(joint);
    ch    = joint_to_channel(joint);
    PCA9685_Cascade_SetPulseMs(cas, (PCA9685_BoardId_t)board, ch, pulse_ms);
}

uint8_t PCA9685_Cascade_GetJointBoard(uint8_t joint)
{
    return joint_to_board(joint);
}

uint8_t PCA9685_Cascade_GetJointChannel(uint8_t joint)
{
    return joint_to_channel(joint);
}