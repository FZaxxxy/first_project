/**
 * @file    cmd_parser.c
 * @brief   串口指令解析器实现
 * @note    第二阶段 2.3 运动控制层
 */
#include "cmd_parser.h"
#include "joint.h"
#include "action.h"
#include "servo.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ------------------------------------------------------------------
 * 工具函数
 * ------------------------------------------------------------------ */

/* 不区分大小写的字符串比较（代替 strcasecmp，兼容性更好） */
static int icmp(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca += 'a' - 'A';
        if (cb >= 'A' && cb <= 'Z') cb += 'a' - 'A';
        if (ca != cb) return (int)(ca - cb);
        a++; b++;
    }
    return (int)(*a - *b);
}

/* 指令帮助文本 */
static void cmd_help(void)
{
    Cmd_Respond("\r\n--- 指令帮助 ---\r\n"
                "HELP            帮助\r\n"
                "HOME            回到初始位姿\r\n"
                "PICK            演示 抓取->旋转->放置\r\n"
                "ACTION <名称>   播放动作\r\n"
                "J <关节> <角度> 单关节定位\r\n"
                "GRIP <0/1>      夹爪开合\r\n"
                "IK <x> <y> <z>  逆运动学移动(m)\r\n"
                "AUTO / MANUAL   智能/手动 模式\r\n"
                "DBG <0..4>      日志级别\r\n");
}

/**
 * @brief  处理一行指令
 *
 * 解析流程：整行拷贝 -> 按空格分词 -> 首词匹配指令 -> 分发。
 */
void CmdParser_ProcessLine(const char *line)
{
    char  buf[CMD_BUF_SIZE];
    char *argv[8];
    int   argc = 0;
    char *tok;

    if (line == 0 || *line == 0) return;

    /* 1. 拷贝到本地缓冲并截断 */
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    /* 2. 分词 */
    tok = strtok(buf, " \t\r\n");
    while (tok && argc < 8) {
        argv[argc++] = tok;
        tok = strtok(0, " \t\r\n");
    }
    if (argc == 0) return;

    /* 3. 指令分发（不区分大小写） */
    if (icmp(argv[0], "HELP") == 0) {
        cmd_help();

    } else if (icmp(argv[0], "HOME") == 0) {
        Action_PlayByName("HOME");

    } else if (icmp(argv[0], "PICK") == 0) {
        Action_DemoPickPlace();

    } else if (icmp(argv[0], "ACTION") == 0) {
        if (argc >= 2) {
            if (Action_FindByName(argv[1]) == 0xFF) {
                Cmd_Respond("未知动作名。用 HELP 查看列表\r\n");
            } else {
                Action_PlayByName(argv[1]);
            }
        } else {
            Cmd_Respond("用法: ACTION <名称>\r\n");
        }

    } else if (icmp(argv[0], "J") == 0) {
        if (argc >= 3) {
            uint8_t j = (uint8_t)atoi(argv[1]);
            float deg = (float)atof(argv[2]);
            if (j < 1 || j > JOINT_NUM) {
                Cmd_Respond("关节号超出范围(1~6)\r\n");
            } else {
                Joint_MoveTo(j, deg, 1000.0f);
            }
        } else {
            Cmd_Respond("用法: J <关节> <角度>\r\n");
        }

    } else if (icmp(argv[0], "GRIP") == 0) {
        if (argc >= 2) {
            Servo_SetGripper((uint8_t)(atoi(argv[1]) ? 1u : 0u));
        } else {
            Cmd_Respond("用法: GRIP <0/1>\r\n");
        }

    } else if (icmp(argv[0], "IK") == 0) {
        if (argc >= 4) {
            float x = (float)atof(argv[1]);
            float y = (float)atof(argv[2]);
            float z = (float)atof(argv[3]);
            Cmd_OnIK(x, y, z);
        } else {
            Cmd_Respond("用法: IK <x> <y> <z>\r\n");
        }

    } else if (icmp(argv[0], "AUTO") == 0) {
        Cmd_OnAuto();

    } else if (icmp(argv[0], "MANUAL") == 0) {
        Cmd_OnManual();

    } else if (icmp(argv[0], "DBG") == 0) {
        if (argc >= 2) {
            Cmd_OnSetLogLevel((uint8_t)atoi(argv[1]));
        } else {
            Cmd_Respond("用法: DBG <0..4>\r\n");
        }

    } else {
        Cmd_Respond("未知指令，输入 HELP 查看帮助\r\n");
    }
}

/* ==================== 弱函数默认实现 ==================== */

__weak void Cmd_Respond(const char *msg)
{
    (void)msg;   /* 默认无输出，由上层(ui.c)覆写为串口发送 */
}

__weak void Cmd_OnIK(float x, float y, float z)
{
    (void)x; (void)y; (void)z;
    Cmd_Respond("IK 功能将在第三阶段实现\r\n");
}

__weak void Cmd_OnAuto(void)
{
    Cmd_Respond("智能模式将在第三/四阶段实现\r\n");
}

__weak void Cmd_OnManual(void)
{
    Cmd_Respond("已返回手动模式\r\n");
}

__weak void Cmd_OnSetLogLevel(uint8_t level)
{
    (void)level;
    Cmd_Respond("日志系统将在第四阶段实现\r\n");
}