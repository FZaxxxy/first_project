/**
 * @file    stm32f1xx_it.h
 * @brief   中断服务函数声明
 * @note    第四阶段 4.1 系统集成
 */
#ifndef __STM32F1XX_IT_H
#define __STM32F1XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM2_IRQHandler(void);
void USART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1XX_IT_H */