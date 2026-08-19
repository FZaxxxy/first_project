/**
 * @file    main.h
 * @brief   主程序头文件
 * @note    第四阶段 4.1 系统集成
 *
 * 外设句柄声明（在 main.c 中定义）。
 * 若你在 CubeMX 中使用了不同的外设名/通道，请同步修改本文件与 config.h。
 */
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

/* ==================== 外设句柄 ==================== */
extern I2C_HandleTypeDef   hi2c1;    /* I2C1：连接两片 PCA9685 */
extern UART_HandleTypeDef  huart1;   /* USART1：串口指令 + 日志 */
extern TIM_HandleTypeDef   htim2;    /* TIM2：1ms 运动控制节拍 */

/** 系统时钟配置（CubeMX 生成，示例：HSI 8MHz -> PLL x9 -> 72MHz） */
void SystemClock_Config(void);

/** 全局错误处理 */
void Error_Handler(void);

#endif /* __MAIN_H */