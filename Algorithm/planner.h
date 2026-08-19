/**
 * @file    planner.h
 * @brief   路径规划与闭环控制
 * @note    第三阶段 3.3 智能算法层
 *
 * 自主抓取流程（非阻塞，由 Planner_Tick 驱动各阶段）：
 *
 *   APPROACH -> DESCEND -> GRASP -> LIFT -> TRANSFER -> PLACE -> RELEASE -> RETURN
 *
 * 安全策略：
 *   - 先上升到安全高度再平移，避免夹爪与桌面/障碍碰撞；
 *   - 抓取后用"夹爪限位开关 / 光电传感器"闭环确认，失败自动重试。
 */
#ifndef __PLANNER_H
#define __PLANNER_H

#include "config.h"

/* ==================== 状态定义 ==================== */
typedef enum {
    PLANNER_IDLE = 0,      /* 空闲 */
    PLANNER_RUNNING,       /* 执行中 */
    PLANNER_DONE,          /* 成功完成 */
    PLANNER_ERROR,         /* 失败 */
} Planner_State_t;

typedef enum {
    PHASE_IDLE = 0,
    PHASE_APPROACH,        /* 上升到目标上方安全高度 */
    PHASE_DESCEND,         /* 下降到抓取点 */
    PHASE_GRASP,           /* 夹爪闭合（含确认） */
    PHASE_LIFT,            /* 抓起后抬升 */
    PHASE_TRANSFER,        /* 平移到放置点上方 */
    PHASE_PLACE,           /* 下降到放置点 */
    PHASE_RELEASE,         /* 张开夹爪 */
    PHASE_RETURN,          /* 返回初始位姿 */
} Planner_Phase_t;

/* ==================== 对外接口 ==================== */

/** 初始化 */
void Planner_Init(void);

/** 启动一次自主抓取（从视觉获取目标，非阻塞） */
void Planner_StartPick(void);

/** 周期调用（放入主循环）：推进各阶段 */
void Planner_Tick(void);

/** 单点运动：平滑移动到基座坐标 (x, y, z)，阻塞执行
 *  @return 1=成功  0=IK 无解或移动失败 */
int Planner_GotoXYZ(float x, float y, float z);

/** 查询状态与当前阶段 */
Planner_State_t Planner_GetState(void);
Planner_Phase_t Planner_GetPhase(void);
const char    *Planner_GetPhaseName(void);

#endif /* __PLANNER_H */