/**
 * @file    ui.c
 * @brief   按键 / 串口交互实现
 * @note    第四阶段 4.2 系统集成
 *
 * 【并发设计】
 * - 串口中断只负责"收字节 + 组行"，指令解析放到主循环
 *   （UI_ProcessPendingCommand），避免阻塞性指令卡死中断；
 * - 按键短按切换模式，长按产生 PICK 演示请求，同样由主循环执行。
 */
#include "ui.h"
#include "debug.h"
#include "cmd_parser.h"
#include "planner.h"
#include "app.h"
#include <string.h>

/* 长按判定阈值（ms） */
#define LONG_PRESS_MS      1500u

static Button_t g_btn_mode;
static uint8_t  g_rx_byte;                     /* 串口单字节缓冲 */
static uint8_t  g_line_buf[CMD_BUF_SIZE];      /* 行组装缓冲 */
static uint16_t g_line_len = 0;

static uint8_t  g_cmd_buf[CMD_BUF_SIZE];       /* 待处理指令 */
static uint8_t  g_cmd_ready = 0;

static uint8_t  g_long_press_sent = 0;         /* 长按仅触发一次 */
static uint8_t  g_demo_request = 0;            /* PICK 演示请求 */

uint8_t *UI_UART_GetRxBytePtr(void)
{
    return &g_rx_byte;
}

/* ------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------ */
void UI_Init(void)
{
    Button_Init(&g_btn_mode, BTN_MODE_PORT, BTN_MODE_PIN);

    g_line_len = 0;
    g_cmd_ready = 0;
    g_demo_request = 0;
    g_long_press_sent = 0;
    memset(g_line_buf, 0, sizeof(g_line_buf));

    /* 启动串口单字节中断接收 */
    HAL_UART_Receive_IT(DBG_HUART, &g_rx_byte, 1);
}

/* ------------------------------------------------------------------
 * 周期任务：按键扫描（放入定时器节拍）
 * ------------------------------------------------------------------ */
void UI_Tick(void)
{
    Button_Scan(&g_btn_mode);

    /* 短按：手动 <-> 智能 模式切换 */
    if (Button_JustPressed(&g_btn_mode)) {
        App_ToggleMode();
        g_long_press_sent = 0;
        return;
    }

    /* 长按：产生 PICK 演示请求（主循环执行） */
    if (Button_IsPressed(&g_btn_mode) &&
        Button_PressTime(&g_btn_mode) > LONG_PRESS_MS) {
        if (!g_long_press_sent) {
            g_long_press_sent = 1;
            g_demo_request = 1;
            DBG_I("长按触发: PICK 演示请求");
        }
    } else if (!Button_IsPressed(&g_btn_mode)) {
        g_long_press_sent = 0;   /* 松开后复位，允许下次长按 */
    }
}

/* ------------------------------------------------------------------
 * 串口接收（中断回调中调用：仅组行，不解析）
 * ------------------------------------------------------------------ */
void UI_UART_RxChar(uint8_t c)
{
    /* 重新开启下一次接收 */
    HAL_UART_Receive_IT(DBG_HUART, &g_rx_byte, 1);

    if (c == '\n') {
        /* 一行结束：暂存为待处理指令（主循环解析） */
        g_line_buf[g_line_len] = 0;
        if (!g_cmd_ready) {
            memcpy(g_cmd_buf, g_line_buf, g_line_len + 1);
            g_cmd_ready = 1;
        }
        g_line_len = 0;
        return;
    }

    if (c == '\r') return;   /* 忽略回车 */

    if (g_line_len < (uint16_t)(sizeof(g_line_buf) - 1)) {
        g_line_buf[g_line_len++] = c;
    } else {
        g_line_len = 0;      /* 超长行丢弃 */
    }
}

/* 主循环处理一条指令 */
uint8_t UI_ProcessPendingCommand(void)
{
    if (!g_cmd_ready) return 0;
    CmdParser_ProcessLine((const char *)g_cmd_buf);
    g_cmd_ready = 0;
    return 1;
}

uint8_t UI_GetPendingDemo(void)
{
    return g_demo_request;
}

void UI_ClearPendingDemo(void)
{
    g_demo_request = 0;
}

void UI_SendString(const char *s)
{
    if (s == 0) return;
    HAL_UART_Transmit(DBG_HUART, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

/* ------------------------------------------------------------------
 * 覆写 cmd_parser 弱函数（接入应用逻辑）
 * ------------------------------------------------------------------ */
void Cmd_Respond(const char *msg)
{
    UI_SendString(msg);
}

void Cmd_OnIK(float x, float y, float z)
{
    if (Planner_GotoXYZ(x, y, z)) {
        Cmd_Respond("IK: 到达目标\r\n");
    } else {
        Cmd_Respond("IK: 无解或目标不可达\r\n");
    }
}

void Cmd_OnAuto(void)
{
    App_StartAuto();
}

void Cmd_OnManual(void)
{
    App_SetState(APP_MANUAL);
}

void Cmd_OnSetLogLevel(uint8_t level)
{
    DBG_SetLevel(level);
    DBG_I("日志级别已设为 %d", DBG_GetLevel());
}