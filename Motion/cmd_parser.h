/**
 * @file    cmd_parser.h
 * @brief   串口指令解析器
 * @note    第二阶段 2.3 运动控制层
 *
 * 文本协议（每行一条指令，空格分隔，\r\n 或 \n 结尾）：
 *
 *   HELP             显示帮助
 *   HOME             回到初始位姿
 *   PICK             演示 抓取->旋转->放置
 *   ACTION <名称>    播放指定动作（见 action.h）
 *   J <关节> <角度>  单关节定位，如  J 2 45
 *   GRIP <0/1>       夹爪开合，如   GRIP 1
 *   IK <x> <y> <z>   逆运动学解算并移动（单位 m）
 *   AUTO             进入智能自主抓取模式
 *   MANUAL           返回手动模式
 *   DBG <0..4>       设置日志级别
 *
 * 指令不区分大小写。上层可通过覆写 Cmd_* 弱函数接入业务逻辑。
 */
#ifndef __CMD_PARSER_H
#define __CMD_PARSER_H

#include "config.h"

/* 兼容不同编译器的弱符号定义（HAL 环境已自带 __weak） */
#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* ==================== 解析入口 ==================== */

/** 处理一行指令（由串口接收回调调用，line 须以 '\0' 结尾） */
void CmdParser_ProcessLine(const char *line);

/* ==================== 上层可选覆写的弱函数 ==================== */

/** 串口应答/提示输出（默认空，第四阶段 ui.c 实现为串口发送） */
__weak void Cmd_Respond(const char *msg);

/** IK 指令：解算并移动到 (x, y, z)（单位 m） */
__weak void Cmd_OnIK(float x, float y, float z);

/** 进入智能自主模式 */
__weak void Cmd_OnAuto(void);

/** 返回手动模式 */
__weak void Cmd_OnManual(void);

/** 设置日志级别 0~4 */
__weak void Cmd_OnSetLogLevel(uint8_t level);

#endif /* __CMD_PARSER_H */