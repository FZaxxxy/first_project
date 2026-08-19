/**
 * @file    stm32f1xx_hal_conf.h
 * @brief   HAL 配置：启用本项目所需外设模块
 *
 * 【重要】本文件是项目自带的 HAL 配置，通过"include 路径优先级"
 * 覆盖 CubeMX 工程(D:/stm32files/pro1)中的同名文件：
 *   - 编译/IntelliSense 时本目录(Core/Inc)必须排在 pro1/Core/Inc 之前；
 *   - 若使用 CubeMX 重新生成工程，请在 CubeMX 中同样勾选
 *     I2C / TIM / USART 三个外设，以便与本配置保持一致。
 *
 * 本文件启用了机械臂控制所需的模块：
 *   GPIO / EXTI / DMA / FLASH / PWR / RCC / CORTEX（基础）
 *   I2C  / TIM  / UART（本项目新增）
 */
#ifndef __STM32F1xx_HAL_CONF_H
#define __STM32F1xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* ==================== 模块使能 ==================== */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ==================== 振荡器参数 ==================== */
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    8000000U   /*!< 外部高速晶振 8MHz */
#endif
#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    100U
#endif
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    8000000U   /*!< 内部高速晶振 8MHz */
#endif
#if !defined  (LSI_VALUE)
  #define LSI_VALUE               40000U
#endif
#if !defined  (LSE_VALUE)
  #define LSE_VALUE    32768U
#endif
#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    5000U
#endif

/* ==================== 系统配置 ==================== */
#define  VDD_VALUE                    3300U
#define  TICK_INT_PRIORITY            15U
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U

#define  USE_HAL_I2C_REGISTER_CALLBACKS   0U
#define  USE_HAL_TIM_REGISTER_CALLBACKS   0U
#define  USE_HAL_UART_REGISTER_CALLBACKS  0U

/* ==================== 断言 ==================== */
/* #define USE_FULL_ASSERT    1U */

/* ==================== 模块头文件 ==================== */
#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f1xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f1xx_hal_gpio.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f1xx_hal_exti.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f1xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f1xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f1xx_hal_flash.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32f1xx_hal_pwr.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
  #include "stm32f1xx_hal_i2c.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32f1xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32f1xx_hal_uart.h"
#endif

/* ==================== 断言宏 ==================== */
#ifdef  USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_CONF_H */