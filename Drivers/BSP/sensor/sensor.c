/**
 * @file    sensor.c
 * @brief   传感器接口实现
 * @note    第三阶段 3.2 支持模块（BSP 层）
 */
#include "sensor.h"

static Sensor_Vision_t g_vision = {0, 0.0f, 0.0f, 0.0f, 0};

void Sensor_Init(void)
{
    /* GPIO 输入模式已在 CubeMX 中配置，此处仅初始化视觉缓存 */
    g_vision.valid = 0;
    g_vision.stamp = 0;
}

uint8_t Sensor_PhotoDetected(void)
{
    /* 低电平有效（常见 NPN 光电传感器），★ 按实际接线调整 */
    return (HAL_GPIO_ReadPin(SENSOR_PHOTO_PORT, SENSOR_PHOTO_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
}

uint8_t Sensor_GripClosed(void)
{
    /* 低电平有效，★ 按实际接线调整 */
    return (HAL_GPIO_ReadPin(SENSOR_GRIP_PORT, SENSOR_GRIP_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
}

void Sensor_UpdateVision(uint8_t valid, float px, float py, float size)
{
    g_vision.valid = valid;
    g_vision.px    = px;
    g_vision.py    = py;
    g_vision.size  = size;
    g_vision.stamp = HAL_GetTick();
}

void Sensor_GetVision(Sensor_Vision_t *v)
{
    if (v == 0) return;
    *v = g_vision;
}