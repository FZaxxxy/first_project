/**
 * @file    debug.h
 * @brief   调试与日志系统
 * @note    第四阶段 4.3 系统集成
 *
 * 轻量级串口日志：分级过滤 + 时间无关的纯文本输出。
 * 宏调用示例：
 *   DBG_E("I2C 初始化失败");          // 错误
 *   DBG_W("关节 %d 超出限位", 3);      // 警告
 *   DBG_I("模式切换为 %s", name);      // 信息
 *   DBG_D("角度 %.2f", deg);           // 调试
 *
 * 注意：使用浮点格式 %f 时，链接参数需加 -u _printf_float
 *       （否则浮点显示为 0）。
 */
#ifndef __DEBUG_H
#define __DEBUG_H

#include "config.h"
#include <stdarg.h>

/* ==================== 日志级别 ==================== */
typedef enum {
    DBG_NONE  = 0,   /* 关闭全部输出 */
    DBG_ERROR = 1,   /* 仅错误 */
    DBG_WARN  = 2,   /* 错误 + 警告 */
    DBG_INFO  = 3,   /* 错误 + 警告 + 信息 */
    DBG_DEBUG = 4,   /* 全部（含调试） */
} DBG_Level_t;

/* ==================== 对外接口 ==================== */

/** 初始化（使用 config.h 中的默认级别） */
void DBG_Init(void);

/** 设置日志级别 0~4 */
void DBG_SetLevel(uint8_t level);

/** 读取当前级别 */
uint8_t DBG_GetLevel(void);

/** 带级别打印（内部使用，一般用下面宏即可） */
void DBG_Print(DBG_Level_t level, const char *fmt, ...);

/* ==================== 快捷宏 ==================== */
#define DBG_E(fmt, ...)  DBG_Print(DBG_ERROR, "[E] " fmt "\r\n", ##__VA_ARGS__)
#define DBG_W(fmt, ...)  DBG_Print(DBG_WARN,  "[W] " fmt "\r\n", ##__VA_ARGS__)
#define DBG_I(fmt, ...)  DBG_Print(DBG_INFO,  "[I] " fmt "\r\n", ##__VA_ARGS__)
#define DBG_D(fmt, ...)  DBG_Print(DBG_DEBUG, "[D] " fmt "\r\n", ##__VA_ARGS__)

#endif /* __DEBUG_H */