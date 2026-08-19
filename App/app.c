/**
 * @file    app.c
 * @brief   主程序状态机实现
 * @note    第四阶段 4.1 系统集成
 */
#include "app.h"
#include "debug.h"
#include "ui.h"
#include "joint.h"
#include "action.h"
#include "sensor.h"
#include "vision.h"
#include "planner.h"

static App_State_t g_state = APP_INIT;

const char *App_GetStateName(void)
{
    switch (g_state) {
    case APP_INIT:   return "INIT";
    case APP_IDLE:   return "IDLE";
    case APP_MANUAL: return "MANUAL";
    case APP_AUTO:   return "AUTO";
    default:         return "?";
    }
}

void App_Init(void)
{
    /* 各层初始化（顺序：硬件 -> 运动 -> 算法 -> 交互） */
    DBG_Init();
    Joint_Init();            /* 内部完成 PCA9685 级联 + 舵机初始化 */
    Sensor_Init();
    Vision_Init();
    Planner_Init();
    UI_Init();

    /* 回到初始位姿（夹爪张开） */
    Action_PlayByName("HOME");
    Action_WaitDone(0);

    g_state = APP_IDLE;
    DBG_I("系统初始化完成，模式: %s", App_GetStateName());
}

void App_SetState(App_State_t s)
{
    if (s == g_state) return;
    g_state = s;
    DBG_I("模式切换 -> %s", App_GetStateName());

    if (s == APP_AUTO) {
        App_StartAuto();     /* 进入自动即开始规划 */
    }
}

void App_StartAuto(void)
{
    g_state = APP_AUTO;
    DBG_I("启动智能自主抓取");
    Planner_StartPick();
}

void App_ToggleMode(void)
{
    App_SetState((g_state == APP_AUTO) ? APP_MANUAL : APP_AUTO);
}

void App_Task(void)
{
    /* 处理串口收齐的指令（阻塞性指令也在主循环执行） */
    if (UI_ProcessPendingCommand()) {
        return;
    }

    /* 处理长按触发的 PICK 演示请求（手动模式下） */
    if (g_state == APP_MANUAL && UI_GetPendingDemo()) {
        UI_ClearPendingDemo();
        Action_DemoPickPlace();
        return;
    }

    /* 自动模式下驱动规划器推进各阶段 */
    if (g_state == APP_AUTO) {
        Planner_Tick();

        /* 规划结束状态汇报 */
        if (Planner_GetState() == PLANNER_DONE) {
            DBG_I("自主抓取成功完成");
            App_SetState(APP_MANUAL);
        } else if (Planner_GetState() == PLANNER_ERROR) {
            DBG_E("自主抓取失败（阶段 %s）", Planner_GetPhaseName());
            App_SetState(APP_MANUAL);
        }
    }
}