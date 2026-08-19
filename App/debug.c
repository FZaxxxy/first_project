/**
 * @file    debug.c
 * @brief   调试与日志系统实现
 * @note    第四阶段 4.3 系统集成
 */
#include "debug.h"
#include <stdio.h>
#include <string.h>

static uint8_t g_level = DBG_DEFAULT_LEVEL;

void DBG_Init(void)
{
    g_level = DBG_DEFAULT_LEVEL;
}

void DBG_SetLevel(uint8_t level)
{
    if (level > DBG_DEBUG) level = DBG_DEBUG;
    g_level = level;
}

uint8_t DBG_GetLevel(void)
{
    return g_level;
}

void DBG_Print(DBG_Level_t level, const char *fmt, ...)
{
    char     buf[256];
    va_list  ap;
    uint16_t len;

    if (level > g_level) return;          /* 级别过滤 */

    va_start(ap, fmt);
    len = (uint16_t)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len > (uint16_t)(sizeof(buf) - 1)) len = (uint16_t)(sizeof(buf) - 1);
    HAL_UART_Transmit(DBG_HUART, (uint8_t *)buf, len, 100);
}