/**
 * @file    ik.h
 * @brief   反向运动学（IK）求解器
 * @note    第三阶段 3.1 智能算法层
 *
 * 实现思路（数值法，适用于任意 DH 结构的 6 自由度机械臂）：
 *   1. 正运动学(FK)：由 DH 参数计算末端位姿；
 *   2. 数值雅可比：对每个关节施加微小扰动，差分求出末端速度雅可比 J；
 *   3. 阻尼最小二乘(DLS)：dq = J^T (J J^T + λ²I)⁻¹ e
 *      迭代逼近目标位姿，λ 越大越稳定（牺牲速度）。
 *
 * 该方案不依赖机械臂的封闭解析解，改 DH 参数即可适配实机。
 */
#ifndef __IK_H
#define __IK_H

#include "config.h"

/* ==================== 求解器配置 ==================== */
typedef struct {
    uint8_t mask[6];     /* 约束掩码：1=约束该自由度。
                            [0..2]=位置 x/y/z，[3..5]=姿态 roll/pitch/yaw */
    uint8_t max_iter;    /* 最大迭代次数 */
    float   pos_tol;     /* 位置收敛误差（m） */
    float   ori_tol;     /* 姿态收敛误差（rad） */
    float   lambda;      /* 阻尼正则项 λ */
} IK_Config_t;

/* ==================== 对外接口 ==================== */

/** 默认配置：位置 + 姿态全部约束 */
void IK_DefaultConfig(IK_Config_t *cfg);

/** 通用求解：目标位置(m) + 目标姿态(度，可为 NULL=不约束姿态)
 *  @return 1=成功  0=失败 */
int  IK_Solve(IK_Config_t *cfg,
              const float target_xyz[3],
              const float rpy_deg[3],
              float q_out[JOINT_NUM]);

/** 便捷接口：仅约束末端位置（常用于抓取点定位） */
int  IK_SolvePosition(const float xyz[3], float q_out[JOINT_NUM]);

/** 便捷接口：约束完整位姿（位置 + 姿态） */
int  IK_SolveFull(const float xyz[3], const float rpy_deg[3],
                  float q_out[JOINT_NUM]);

/** 正运动学：由关节角(rad) 求末端位置(m) */
void IK_ForwardKinematics(const float q_rad[JOINT_NUM], float xyz_out[3]);

#endif /* __IK_H */