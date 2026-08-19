/**
 * @file    action.c
 * @brief   多关节协调运动（动作组）实现
 * @note    第二阶段 2.2 运动控制层
 *
 * 协调机制：
 *   1. 计算所有关节中最大的角度变化量 max_delta；
 *   2. 总时长 = max_delta * ACTION_MS_PER_DEG（并限制最短时长）；
 *   3. 所有关节使用相同总时长做平滑运动 -> 同时启动、同时到达。
 */
#include "action.h"
#include "joint.h"
#include "servo.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------
 * 动作组定义表
 *   name: 动作名称
 *   q[]:  6 个关节的目标角度（度）
 *   grip: 夹爪动作  -1=不动作  0=张开  1=闭合
 * ------------------------------------------------------------------ */
typedef struct {
    const char *name;
    float       q[JOINT_NUM];
    int8_t      grip;
} Action_t;

/* ★ 须自定义：以下关节角度为"示教位姿"示例，请按实际机械臂标定修改 */
static const Action_t g_actions[] = {
    {"HOME",          {  0,   0,   0,   0,   0,   0},  0},  /* 初始位姿（夹爪张开） */
    {"PICK_UP",       { 30, -30,  40,   0,  30,   0}, -1},  /* 抓取点上方 */
    {"PICK_DOWN",     { 30, -45,  55,   0,  20,   0}, -1},  /* 下降到抓取目标 */
    {"GRASP",         { 30, -45,  55,   0,  20,   0},  1},  /* 夹爪闭合 */
    {"LIFT",          { 25, -25,  35,   0,  15,   0}, -1},  /* 抬起 */
    {"ROTATE",        { 70, -25,  35,   0,  15,  60}, -1},  /* 旋转（转腰+转腕） */
    {"PLACE_UP",      { 70, -30,  40,   0,  30,  60}, -1},  /* 放置点上方 */
    {"PLACE_DOWN",    { 70, -45,  55,   0,  20,  60}, -1},  /* 下降到放置点 */
    {"RELEASE",       { 70, -45,  55,   0,  20,  60},  0},  /* 夹爪张开 */
    {"PLACE_RETURN",  { 70, -30,  40,   0,  30,  60}, -1},  /* 放置后抬起 */
};

#define ACTION_NUM   (sizeof(g_actions) / sizeof(g_actions[0]))

uint8_t Action_GetCount(void)
{
    return (uint8_t)ACTION_NUM;
}

const char *Action_GetName(uint8_t idx)
{
    if (idx >= ACTION_NUM) return "?";
    return g_actions[idx].name;
}

uint8_t Action_FindByName(const char *name)
{
    uint8_t i;
    if (name == 0) return 0xFF;
    for (i = 0; i < ACTION_NUM; i++) {
        if (strcmp(g_actions[i].name, name) == 0) {
            return i;
        }
    }
    return 0xFF;
}

/**
 * @brief  播放动作（协调运动，非阻塞）
 */
void Action_Play(uint8_t idx)
{
    const Action_t *act;
    float max_delta = 0.0f;
    float duration;
    uint8_t j;

    if (idx >= ACTION_NUM) return;
    act = &g_actions[idx];

    /* 计算最大关节变化量，用于确定协调总时长 */
    for (j = 0; j < JOINT_NUM; j++) {
        float delta = fabsf(act->q[j] - Joint_GetAngle(j + 1));
        if (delta > max_delta) max_delta = delta;
    }

    duration = max_delta * ACTION_MS_PER_DEG;
    if (duration < ACTION_MIN_DURATION_MS) duration = ACTION_MIN_DURATION_MS;

    /* 所有关节同时启动、按相同总时长运动 */
    for (j = 0; j < JOINT_NUM; j++) {
        Joint_MoveTo(j + 1, act->q[j], duration);
    }

    /* 夹爪动作（若本动作需要） */
    if (act->grip >= 0) {
        Servo_SetGripper((uint8_t)act->grip);
    }
}

void Action_PlayByName(const char *name)
{
    uint8_t idx = Action_FindByName(name);
    if (idx != 0xFF) {
        Action_Play(idx);
    }
}

uint8_t Action_IsBusy(void)
{
    uint8_t j;
    for (j = 0; j < JOINT_NUM; j++) {
        if (Joint_IsMoving(j + 1)) return 1;
    }
    return 0;
}

void Action_WaitDone(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (Action_IsBusy()) {
        if (timeout_ms > 0 && (HAL_GetTick() - t0) > timeout_ms) break;
        HAL_Delay(MOTION_TICK_MS);
    }
}

/**
 * @brief  演示完整流程：抓取 -> 旋转 -> 放置（阻塞）
 */
void Action_DemoPickPlace(void)
{
    Action_PlayByName("HOME");
    Action_WaitDone(0);
    Action_PlayByName("PICK_UP");
    Action_WaitDone(0);
    Action_PlayByName("PICK_DOWN");
    Action_WaitDone(0);
    Action_PlayByName("GRASP");
    HAL_Delay(300);                     /* 留时间给夹爪闭合 */
    Action_PlayByName("LIFT");
    Action_WaitDone(0);
    Action_PlayByName("ROTATE");
    Action_WaitDone(0);
    Action_PlayByName("PLACE_UP");
    Action_WaitDone(0);
    Action_PlayByName("PLACE_DOWN");
    Action_WaitDone(0);
    Action_PlayByName("RELEASE");
    HAL_Delay(300);
    Action_PlayByName("PLACE_RETURN");
    Action_WaitDone(0);
    Action_PlayByName("HOME");
    Action_WaitDone(0);
}