/**
 * @file    stm32f1xx_it.c
 * @brief   中断服务程序（模板）
 * @note    第四阶段 4.1 系统集成
 *
 * 【集成说明】本文件为 CubeMX 生成版本的参考模板。
 * 若你使用 CubeMX 生成工程，请以生成版本为准，并确保包含
 * TIM2_IRQHandler 与 USART1_IRQHandler 即可。
 */
#include "main.h"
#include "stm32f1xx_it.h"

/** 系统滴答中断 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/** 定时器 2 中断：1ms 节拍 -> HAL_TIM_PeriodElapsedCallback */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/** 串口 1 中断：接收/发送 -> HAL_UART_RxCpltCallback 等 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* 以下为 STM32F1 常见中断占位（按实际使用情况保留） */
void NMI_Handler(void)          { }
void HardFault_Handler(void)    { while (1) { } }
void MemManage_Handler(void)    { while (1) { } }
void BusFault_Handler(void)     { while (1) { } }
void UsageFault_Handler(void)   { while (1) { } }
void DebugMon_Handler(void)     { }
void PendSV_Handler(void)       { }