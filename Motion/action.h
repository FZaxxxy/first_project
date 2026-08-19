/**
 * @file    action.h
 * @brief   多关节协调运动（动作组）
 * @note    第二阶段 2.2 运动控制层
 *
 * 动作组 = 一组关节目标角度的集合，一次调用即可让所有关节
 * "同时启动、同时到达"，实现机械臂多自由度协调运动。
 *
 * 预置动作（角度单位：度，夹爪值 -1=不动作 0=张开 1=闭合）：
 *   HOME / PICK_UP / PICK_DOWN / GRASP / LIFT /
 *   ROTATE / PLACE_UP / PLACE_DOWN / RELEASE / PLACE_RETURN
 * 以上动作对应"抓取 -> 旋转 -> 放置"完整流程。
 */
#ifndef __ACTION_H
#define __ACTION_H

#include "config.h"

/* ==================== 对外接口 ==================== */

/** 动作组总数 */
uint8_t Action_GetCount(void);

/** 获取第 idx 个动作组的名称（用于串口/按键提示） */
const char *Action_GetName(uint8_t idx);

/** 按名称查找动作组，返回下标；找不到返回 0xFF */
uint8_t Action_FindByName(const char *name);

/** 播放第 idx 个动作（协调运动，非阻塞，内部由节拍驱动） */
void Action_Play(uint8_t idx);

/** 按名称播放动作 */
void Action_PlayByName(const char *name);

/** 阻塞等待当前动作完成（最多等待 timeout_ms，0=无限） */
void Action_WaitDone(uint32_t timeout_ms);

/** 当前是否有动作在执行 */
uint8_t Action_IsBusy(void);

/** 演示：抓取 -> 旋转 -> 放置 完整流程（阻塞） */
void Action_DemoPickPlace(void);

#endif /* __ACTION_H */