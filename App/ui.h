/**
 * @file    ui.h
 * @brief   按键 / 串口交互
 * @note    第四阶段 4.2 系统集成
 *
 * 功能：
 *   - 串口收包：中断接收单字节 -> 行缓冲 -> 交给指令解析器；
 *   - 串口应答：实现 cmd_parser 的 Cmd_Respond 等弱函数；
 *   - 按键交互：短按切换 手动/智能 模式，长按触发 PICK 演示；
 *   - OpenMV 数据上报解析（例如 "V 80 60 25\n"）。
 */
#ifndef __UI_H
#define __UI_H

#include "config.h"
#include "button.h"

/* ==================== 对外接口 ==================== */

/** 初始化 UI（按键、串口中断接收启动） */
void UI_Init(void);

/** 周期调用（放入定时器节拍）：按键扫描 */
void UI_Tick(void);

/** 由串口接收中断调用：接收一个字节（仅缓存，主循环再解析） */
void UI_UART_RxChar(uint8_t c);

/** 返回内部单字节接收缓冲地址（供串口中断回调使用） */
uint8_t *UI_UART_GetRxBytePtr(void);

/** 主循环调用：处理一条已收完的指令，有则返回 1 */
uint8_t UI_ProcessPendingCommand(void);

/** 查询是否有待执行的 PICK 演示请求（长按触发） */
uint8_t UI_GetPendingDemo(void);

/** 清除演示请求 */
void UI_ClearPendingDemo(void);

/** 串口发送字符串（供 Cmd_Respond 使用） */
void UI_SendString(const char *s);

#endif /* __UI_H */