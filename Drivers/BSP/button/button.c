/**
 * @file    button.c
 * @brief   按键驱动实现
 * @note    第四阶段 4.2 系统集成
 */
#include "button.h"

void Button_Init(Button_t *btn, GPIO_TypeDef *port, uint16_t pin)
{
    if (btn == 0) return;
    btn->port         = port;
    btn->pin          = pin;
    btn->last         = GPIO_PIN_SET;   /* 默认松开（高电平） */
    btn->cnt          = 0;
    btn->stable       = GPIO_PIN_SET;
    btn->just_pressed = 0;
    btn->down_tick    = 0;
}

void Button_Scan(Button_t *btn)
{
    uint8_t lv;

    if (btn == 0) return;
    lv = HAL_GPIO_ReadPin(btn->port, btn->pin);

    if (lv != btn->last) {
        /* 电平跳变：重新计数 */
        btn->last = lv;
        btn->cnt  = 0;
        return;
    }

    if (btn->cnt < BUTTON_DEBOUNCE_CNT) {
        btn->cnt++;
        return;
    }

    /* 电平连续稳定 BUTTON_DEBOUNCE_CNT 次，确认有效 */
    if (btn->stable != lv) {
        btn->stable = lv;
        if (lv == GPIO_PIN_RESET) {            /* 确认按下（低有效） */
            btn->just_pressed = 1;
            btn->down_tick    = HAL_GetTick();
        }
    }
}

uint8_t Button_IsPressed(Button_t *btn)
{
    if (btn == 0) return 0;
    return (btn->stable == GPIO_PIN_RESET) ? 1u : 0u;
}

uint8_t Button_JustPressed(Button_t *btn)
{
    uint8_t r;
    if (btn == 0) return 0;
    r = btn->just_pressed;
    btn->just_pressed = 0;
    return r;
}

uint32_t Button_PressTime(Button_t *btn)
{
    if (btn == 0 || !Button_IsPressed(btn)) return 0;
    return HAL_GetTick() - btn->down_tick;
}