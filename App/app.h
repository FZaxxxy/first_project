/**
 * @file    app.h
 * @brief   主程序状态机
 * @note    第四阶段 4.1 系统集成
 *
 * 状态说明：
 *   APP_INIT   初始化（各模块上电自检）
 *   APP_IDLE   空闲待命
 *   APP_MANUAL 手动模式：按键/串口触发预编程动作
 *   APP_AUTO   智能模式：视觉 + IK + 路径规划的自主抓取
 */
#ifndef __APP_H
#define __APP_H

#include "config.h"

/* ==================== 状态定义 ==================== */
typedef enum {
    APP_INIT = 0,
    APP_IDLE,
    APP_MANUAL,
    APP_AUTO,
} App_State_t;

/* ==================== 对外接口 ==================== */

/** 系统初始化：时钟外设由 CubeMX 生成，此处初始化业务模块 */
void App_Init(void);

/** 主循环任务：周期调用 */
void App_Task(void);

/** 查询当前状态与名称 */
App_State_t App_GetState(void);
const char *App_GetStateName(void);

/** 进入指定状态（手动/自动） */
void App_SetState(App_State_t s);

/** 手动 <-> 智能 模式切换（按键用） */
void App_ToggleMode(void);

/** 启动智能自主抓取（自动模式入口） */
void App_StartAuto(void);

#endif /* __APP_H */