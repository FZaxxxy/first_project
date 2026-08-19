/**
 * @file    button.h
 * @brief   按键驱动（GPIO + 软件消抖）
 * @note    第四阶段 4.2 系统集成
 *
 * 采用轮询 + 连续稳定计数消抖，需在周期节拍(如 10ms)中
 * 调用 Button_Scan()。低电平有效（按键按下接地）。
 */
#ifndef __BUTTON_H
#define __BUTTON_H

#include "config.h"

/* 去抖所需连续稳定采样次数（每次 = MOTION_TICK_MS） */
#define BUTTON_DEBOUNCE_CNT   3u

typedef struct {
    GPIO_TypeDef *port;       /* 所在 GPIO 口 */
    uint16_t      pin;        /* 引脚 */
    uint8_t       last;       /* 上次采样电平 */
    uint8_t       cnt;        /* 稳定计数 */
    uint8_t       stable;     /* 去抖后的稳定电平（0=按下 1=松开） */
    uint8_t       just_pressed;   /* 刚按下（供消费，读取后清零） */
    uint32_t      down_tick;      /* 按下时刻（HAL_GetTick） */
} Button_t;

/* ==================== 对外接口 ==================== */

/** 初始化按键（绑定 GPIO） */
void Button_Init(Button_t *btn, GPIO_TypeDef *port, uint16_t pin);

/** 周期调用（每 MOTION_TICK_MS 一次）：采样 + 消抖 */
void Button_Scan(Button_t *btn);

/** 读取去抖后电平：1=按下 */
uint8_t Button_IsPressed(Button_t *btn);

/** 查询"刚按下"边沿（读取后自动清零） */
uint8_t Button_JustPressed(Button_t *btn);

/** 当前已按住时长（ms），未按住返回 0 */
uint32_t Button_PressTime(Button_t *btn);

#endif /* __BUTTON_H */