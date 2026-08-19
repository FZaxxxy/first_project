/**
 * @file    planner.c
 * @brief   路径规划与闭环控制实现
 * @note    第三阶段 3.3 智能算法层
 */
#include "planner.h"
#include "ik.h"
#include "vision.h"
#include "sensor.h"
#include "joint.h"
#include "action.h"
#include "servo.h"
#include <math.h>
#include <string.h>

#define RAD2DEG 57.2957795131f
#define DEG2RAD 0.01745329252f

/* 阶段名称表（用于日志/串口显示） */
static const char *const g_phase_names[] = {
    "IDLE", "APPROACH", "DESCEND", "GRASP", "LIFT",
    "TRANSFER", "PLACE", "RELEASE", "RETURN",
};

/* 内部状态 */
static Planner_State_t g_state   = PLANNER_IDLE;
static Planner_Phase_t g_phase   = PHASE_IDLE;
static float  g_tx, g_ty, g_tz;              /* 目标抓取点 */
static uint8_t g_retry;                       /* 抓取重试计数 */

/* ------------------------------------------------------------------
 * 工具：解算 IK 并协调移动
 * ------------------------------------------------------------------ */

/**
 * @brief  解算到目标点并协调移动（非阻塞）
 * @return 1=IK 有解且已下发运动  0=无解
 */
static int solve_and_move(float x, float y, float z)
{
    float q[JOINT_NUM];
    float target_deg[JOINT_NUM];
    float max_delta = 0.0f, duration;
    int i;

    /* 以当前关节角为初值解位置型 IK */
    for (i = 0; i < JOINT_NUM; i++) {
        q[i] = Joint_GetAngle(i + 1) * DEG2RAD;
    }
    {
        float xyz[3] = {x, y, z};
        if (!IK_SolvePosition(xyz, q)) {
            return 0;
        }
    }

    /* 计算协调运动时长 */
    for (i = 0; i < JOINT_NUM; i++) {
        target_deg[i] = q[i] * RAD2DEG;
        float d = fabsf(target_deg[i] - Joint_GetAngle(i + 1));
        if (d > max_delta) max_delta = d;
    }
    duration = max_delta * ACTION_MS_PER_DEG;
    if (duration < ACTION_MIN_DURATION_MS) duration = ACTION_MIN_DURATION_MS;

    for (i = 0; i < JOINT_NUM; i++) {
        Joint_MoveTo(i + 1, target_deg[i], duration);
    }
    return 1;
}

/* 判断当前是否还有关节在运动 */
static uint8_t motion_done(void)
{
    int i;
    for (i = 0; i < JOINT_NUM; i++) {
        if (Joint_IsMoving(i + 1)) return 0;
    }
    return 1;
}

/* 进入新阶段并打印日志 */
static void set_phase(Planner_Phase_t ph)
{
    g_phase = ph;
}

/* ------------------------------------------------------------------
 * 对外接口
 * ------------------------------------------------------------------ */
void Planner_Init(void)
{
    g_state = PLANNER_IDLE;
    g_phase = PHASE_IDLE;
    g_retry = 0;
}

void Planner_StartPick(void)
{
    float x, y, z;

    if (g_state == PLANNER_RUNNING) return;

    /* 获取视觉目标 */
    if (!Vision_GetTarget(&x, &y, &z)) {
        g_state = PLANNER_ERROR;
        g_phase = PHASE_IDLE;
        return;
    }

    g_tx = x; g_ty = y; g_tz = z;
    g_retry = 0;
    g_state = PLANNER_RUNNING;
    set_phase(PHASE_APPROACH);
}

void Planner_Tick(void)
{
    if (g_state != PLANNER_RUNNING) return;

    switch (g_phase) {

    /* 1. 上升到目标上方安全高度 */
    case PHASE_APPROACH:
        if (!solve_and_move(g_tx, g_ty, TARGET_SAFE_Z_M)) {
            g_state = PLANNER_ERROR;
            break;
        }
        set_phase(PHASE_DESCEND);
        break;

    /* 2. 下降到抓取点 */
    case PHASE_DESCEND:
        if (!motion_done()) break;
        if (!solve_and_move(g_tx, g_ty, g_tz)) {
            g_state = PLANNER_ERROR;
            break;
        }
        set_phase(PHASE_GRASP);
        break;

    /* 3. 夹爪闭合 + 闭环确认（失败重试） */
    case PHASE_GRASP:
        if (!motion_done()) break;
        Servo_SetGripper(0);                         /* 闭合 */
        HAL_Delay(GRASP_CONFIRM_DELAY_MS);
        if (Sensor_GripClosed() || Sensor_PhotoDetected()) {
            set_phase(PHASE_LIFT);                   /* 确认抓住 */
        } else if (g_retry < GRASP_MAX_RETRY) {
            g_retry++;
            Servo_SetGripper(1);                     /* 张开重试 */
            HAL_Delay(200);
            set_phase(PHASE_DESCEND);                /* 重新下降到目标 */
        } else {
            Servo_SetGripper(1);
            g_state = PLANNER_ERROR;                 /* 重试仍失败 */
        }
        break;

    /* 4. 抬起 */
    case PHASE_LIFT:
        if (!motion_done()) break;
        if (!solve_and_move(g_tx, g_ty, TARGET_SAFE_Z_M)) {
            g_state = PLANNER_ERROR;
            break;
        }
        set_phase(PHASE_TRANSFER);
        break;

    /* 5. 平移到放置点上方 */
    case PHASE_TRANSFER:
        if (!motion_done()) break;
        if (!solve_and_move(PLACE_X_M, PLACE_Y_M, TARGET_SAFE_Z_M)) {
            g_state = PLANNER_ERROR;
            break;
        }
        set_phase(PHASE_PLACE);
        break;

    /* 6. 下降到放置点 */
    case PHASE_PLACE:
        if (!motion_done()) break;
        if (!solve_and_move(PLACE_X_M, PLACE_Y_M, g_tz)) {
            g_state = PLANNER_ERROR;
            break;
        }
        set_phase(PHASE_RELEASE);
        break;

    /* 7. 张开夹爪 */
    case PHASE_RELEASE:
        if (!motion_done()) break;
        Servo_SetGripper(1);                         /* 张开 */
        HAL_Delay(300);
        set_phase(PHASE_RETURN);
        break;

    /* 8. 返回初始位姿 */
    case PHASE_RETURN:
        if (!motion_done()) break;
        Action_PlayByName("HOME");
        set_phase(PHASE_IDLE);
        g_state = PLANNER_DONE;
        break;

    default:
        break;
    }
}

int Planner_GotoXYZ(float x, float y, float z)
{
    float q[JOINT_NUM];
    float target_deg[JOINT_NUM];
    float max_delta = 0.0f, duration;
    int i;

    for (i = 0; i < JOINT_NUM; i++) {
        q[i] = Joint_GetAngle(i + 1) * DEG2RAD;
    }
    {
        float xyz[3] = {x, y, z};
        if (!IK_SolvePosition(xyz, q)) return 0;
    }
    for (i = 0; i < JOINT_NUM; i++) {
        target_deg[i] = q[i] * RAD2DEG;
        float d = fabsf(target_deg[i] - Joint_GetAngle(i + 1));
        if (d > max_delta) max_delta = d;
    }
    duration = max_delta * ACTION_MS_PER_DEG;
    if (duration < ACTION_MIN_DURATION_MS) duration = ACTION_MIN_DURATION_MS;
    for (i = 0; i < JOINT_NUM; i++) {
        Joint_MoveTo(i + 1, target_deg[i], duration);
    }
    Action_WaitDone(0);          /* 阻塞等待移动完成 */
    return 1;
}

Planner_State_t Planner_GetState(void)
{
    return g_state;
}

Planner_Phase_t Planner_GetPhase(void)
{
    return g_phase;
}

const char *Planner_GetPhaseName(void)
{
    if (g_phase > PHASE_RETURN) return "?";
    return g_phase_names[g_phase];
}