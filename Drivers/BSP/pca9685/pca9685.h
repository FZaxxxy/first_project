/**
 * @file    pca9685.h
 * @brief   PCA9685 PWM 驱动板驱动（单块）
 * @note    第一阶段 1.1 硬件驱动层
 *
 * 本文件实现单块 PCA9685 的 I2C 底层驱动，提供：
 *   - 复位 / 初始化
 *   - 设置 PWM 频率（默认 50Hz，周期 20ms）
 *   - 按 12 位计数（0~4095）或脉宽（ms）设置任意通道输出
 *
 * 与舵机角度换算（角度 -> 脉宽）无关，请见 servo 模块。
 */
#ifndef __PCA9685_H
#define __PCA9685_H

#include "stm32f1xx_hal.h"

/* ==================== PCA9685 寄存器地址 ==================== */
#define PCA9685_REG_MODE1          0x00u
#define PCA9685_REG_MODE2          0x01u
#define PCA9685_REG_PRESCALE       0xFEu
#define PCA9685_REG_LED0_ON_L      0x06u   /* LED0 开始地址 */
#define PCA9685_REG_ALL_LED_ON_L   0xFAu
#define PCA9685_REG_ALL_LED_OFF_L  0xFCu

/* Mode1 寄存器位定义 */
#define PCA9685_MODE1_RESTART      0x80u
#define PCA9685_MODE1_EXTCLK       0x40u
#define PCA9685_MODE1_AI           0x20u   /* Auto-Increment */
#define PCA9685_MODE1_SLEEP        0x10u
#define PCA9685_MODE1_ALLCALL      0x01u

/* ==================== 单块板设备句柄 ==================== */
typedef struct {
    I2C_HandleTypeDef *hi2c;   /* 所属 I2C 总线句柄 */
    uint8_t            addr;   /* 本板 I2C 地址（如 0x40） */
    uint16_t           freq;   /* PWM 频率（Hz） */
    uint8_t            init_ok;/* 初始化成功标志 */
} PCA9685_Dev_t;

/* ==================== 对外接口 ==================== */
void PCA9685_Init(PCA9685_Dev_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);
void PCA9685_Reset(PCA9685_Dev_t *dev);
void PCA9685_SetPWMFreq(PCA9685_Dev_t *dev, uint16_t freq);
void PCA9685_SetPWMCounts(PCA9685_Dev_t *dev, uint8_t ch, uint16_t on, uint16_t off);
void PCA9685_SetPulseMs(PCA9685_Dev_t *dev, uint8_t ch, float pulse_ms);

/* 内部静态助手（供级联模块复用） */
uint8_t PCA9685_WriteByte(PCA9685_Dev_t *dev, uint8_t reg, uint8_t val);
uint8_t PCA9685_ReadByte(PCA9685_Dev_t *dev, uint8_t reg, uint8_t *val);

#endif /* __PCA9685_H */