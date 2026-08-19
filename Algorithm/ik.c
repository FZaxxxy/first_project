/**
 * @file    ik.c
 * @brief   反向运动学（IK）求解器实现（数值法：雅可比 + 阻尼最小二乘）
 * @note    第三阶段 3.1 智能算法层
 */
#include "ik.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------
 * 机械臂几何参数（来自 config.h，★ 须自定义）
 * ------------------------------------------------------------------ */
static const float g_dh_a[JOINT_NUM]     = DH_A;
static const float g_dh_d[JOINT_NUM]     = DH_D;
static const float g_dh_alpha[JOINT_NUM] = DH_ALPHA;
static const float g_limits[JOINT_NUM][2] = JOINT_LIMITS;

#define RAD2DEG 57.2957795131f
#define DEG2RAD 0.01745329252f
#define PERTURB 0.001f          /* 雅可比差分离散步长(rad) */
#define MAX_STEP 0.5f           /* 单次迭代关节角最大变化(rad) */

/* ------------------------------------------------------------------
 * 基础矩阵运算（4x4 / 3x3 / 线性方程组）
 * ------------------------------------------------------------------ */

/** 4x4 矩阵乘法 out = A·B */
static void mat44_mul(float out[4][4], const float A[4][4], const float B[4][4])
{
    float tmp[4][4];
    int i, j, k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            tmp[i][j] = 0.0f;
            for (k = 0; k < 4; k++) {
                tmp[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    memcpy(out, tmp, sizeof(tmp));
}

/** 3x3 叉积式姿态误差：e = 0.5·(n×n* + s×s* + a×a*) */
static void vec_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

/** 旋转矩阵(按列取 n/s/a 方向向量) 与 目标旋转矩阵的姿态误差 */
static void orientation_error(const float R_cur[3][3], const float R_tgt[3][3],
                              float e[3])
{
    /* 按列提取方向向量 */
    float n_cur[3] = {R_cur[0][0], R_cur[1][0], R_cur[2][0]};
    float s_cur[3] = {R_cur[0][1], R_cur[1][1], R_cur[2][1]};
    float a_cur[3] = {R_cur[0][2], R_cur[1][2], R_cur[2][2]};
    float n_tgt[3] = {R_tgt[0][0], R_tgt[1][0], R_tgt[2][0]};
    float s_tgt[3] = {R_tgt[0][1], R_tgt[1][1], R_tgt[2][1]};
    float a_tgt[3] = {R_tgt[0][2], R_tgt[1][2], R_tgt[2][2]};
    float c[3];

    vec_cross(n_cur, n_tgt, c);   e[0] = c[0]; e[1] = c[1]; e[2] = c[2];
    vec_cross(s_cur, s_tgt, c);   e[0] += c[0]; e[1] += c[1]; e[2] += c[2];
    vec_cross(a_cur, a_tgt, c);   e[0] += c[0]; e[1] += c[1]; e[2] += c[2];
    e[0] *= 0.5f; e[1] *= 0.5f; e[2] *= 0.5f;
}

/** RPY(度) -> 旋转矩阵 R = Rz(yaw)·Ry(pitch)·Rx(roll) */
static void rpy_to_rotation(float roll, float pitch, float yaw, float R[3][3])
{
    float cr = cosf(roll * DEG2RAD), sr = sinf(roll * DEG2RAD);
    float cp = cosf(pitch * DEG2RAD), sp = sinf(pitch * DEG2RAD);
    float cy = cosf(yaw * DEG2RAD),   sy = sinf(yaw * DEG2RAD);

    R[0][0] = cy * cp;           R[0][1] = cy * sp * sr - sy * cr;  R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;           R[1][1] = sy * sp * sr + cy * cr;  R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;               R[2][1] = cp * sr;                 R[2][2] = cp * cr;
}

/* ------------------------------------------------------------------
 * 正运动学（FK）
 * ------------------------------------------------------------------ */

/** 第 j 关节的 DH 变换矩阵（theta 弧度） */
static void dh_transform(float T[4][4], int j, float theta)
{
    float ct = cosf(theta), st = sinf(theta);
    float ca = cosf(g_dh_alpha[j]), sa = sinf(g_dh_alpha[j]);

    T[0][0] = ct;            T[0][1] = -st * ca;     T[0][2] =  st * sa;      T[0][3] = g_dh_a[j] * ct;
    T[1][0] = st;            T[1][1] =  ct * ca;     T[1][2] = -ct * sa;      T[1][3] = g_dh_a[j] * st;
    T[2][0] = 0.0f;          T[2][1] =  sa;          T[2][2] =  ca;           T[2][3] = g_dh_d[j];
    T[3][0] = 0.0f;          T[3][1] =  0.0f;        T[3][2] =  0.0f;         T[3][3] = 1.0f;
}

/** 正运动学：关节角(rad) -> 末端齐次变换 T（含工具偏置） */
static void fk(const float q[JOINT_NUM], float T[4][4])
{
    float Ti[4][4];
    int i;

    /* 初始化为单位阵 */
    for (i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) T[i][j] = (i == j) ? 1.0f : 0.0f;
    }

    /* 依次乘以各关节 DH 变换 */
    for (i = 0; i < JOINT_NUM; i++) {
        dh_transform(Ti, i, q[i]);
        mat44_mul(T, T, Ti);
    }

    /* 末端工具（夹爪）沿 z 轴偏移 */
    float Ttool[4][4];
    memset(Ttool, 0, sizeof(Ttool));
    Ttool[0][0] = Ttool[1][1] = Ttool[2][2] = Ttool[3][3] = 1.0f;
    Ttool[2][3] = TOOL_LENGTH_M;
    mat44_mul(T, T, Ttool);
}

/** 取变换矩阵的旋转矩阵（左上 3x3） */
static void get_rotation(const float T[4][4], float R[3][3])
{
    int i, j;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            R[i][j] = T[i][j];
}

/** 取变换矩阵的平移向量（x, y, z） */
static void get_position(const float T[4][4], float p[3])
{
    p[0] = T[0][3]; p[1] = T[1][3]; p[2] = T[2][3];
}

/* ------------------------------------------------------------------
 * 数值雅可比
 * ------------------------------------------------------------------ */

/**
 * @brief  数值雅可比矩阵 J[6][JOINT_NUM]
 * @param  R_tgt 目标旋转矩阵（姿态误差行需要）
 * @param  e_pos 当前末端位置（位置误差行需要）
 */
static void numeric_jacobian(const float q[JOINT_NUM],
                             const float R_tgt[3][3],
                             float J[6][JOINT_NUM])
{
    int k, i;
    float q_pert[JOINT_NUM];
    float T0[4][4], T1[4][4];
    float p0[3], p1[3], R0[3][3], R1[3][3], e0[3], e1[3];

    for (k = 0; k < JOINT_NUM; k++) {
        memcpy(q_pert, q, sizeof(q_pert));
        q_pert[k] += PERTURB;

        /* 基准位姿 */
        fk(q, T0);
        get_position(T0, p0);
        get_rotation(T0, R0);
        orientation_error(R0, R_tgt, e0);

        /* 扰动位姿 */
        fk(q_pert, T1);
        get_position(T1, p1);
        get_rotation(T1, R1);
        orientation_error(R1, R_tgt, e1);

        /* 位置误差列 */
        for (i = 0; i < 3; i++) {
            J[i][k] = (p1[i] - p0[i]) / PERTURB;
        }
        /* 姿态误差列 */
        for (i = 0; i < 3; i++) {
            J[i + 3][k] = (e1[i] - e0[i]) / PERTURB;
        }
    }
}

/* ------------------------------------------------------------------
 * 线性方程组求解：高斯消元（支持 n<=6）
 * ------------------------------------------------------------------ */
static int gauss_solve(int n, float A[6][6], float b[6])
{
    int k, i, j;

    /* 前向消元 */
    for (k = 0; k < n; k++) {
        /* 选主元 */
        int pivot = k;
        float maxv = fabsf(A[k][k]);
        for (i = k + 1; i < n; i++) {
            if (fabsf(A[i][k]) > maxv) {
                maxv = fabsf(A[i][k]);
                pivot = i;
            }
        }
        if (maxv < 1e-9f) return 0;      /* 奇异矩阵 */

        if (pivot != k) {
            for (j = k; j < n; j++) {
                float t = A[k][j]; A[k][j] = A[pivot][j]; A[pivot][j] = t;
            }
            float t = b[k]; b[k] = b[pivot]; b[pivot] = t;
        }

        for (i = k + 1; i < n; i++) {
            float f = A[i][k] / A[k][k];
            for (j = k; j < n; j++) A[i][j] -= f * A[k][j];
            b[i] -= f * b[k];
        }
    }

    /* 回代 */
    for (i = n - 1; i >= 0; i--) {
        float s = b[i];
        for (j = i + 1; j < n; j++) s -= A[i][j] * b[j];
        b[i] = s / A[i][i];
    }
    return 1;
}

/* ------------------------------------------------------------------
 * 求解器
 * ------------------------------------------------------------------ */
void IK_DefaultConfig(IK_Config_t *cfg)
{
    if (cfg == 0) return;
    cfg->mask[0] = cfg->mask[1] = cfg->mask[2] = 1;   /* 位置 */
    cfg->mask[3] = cfg->mask[4] = cfg->mask[5] = 1;   /* 姿态 */
    cfg->max_iter = IK_MAX_ITER;
    cfg->pos_tol  = IK_POS_TOL_M;
    cfg->ori_tol  = IK_ORI_TOL_RAD;
    cfg->lambda   = IK_DAMPING_LAMBDA;
}

int IK_Solve(IK_Config_t *cfg,
             const float target_xyz[3],
             const float rpy_deg[3],
             float q_out[JOINT_NUM])
{
    float q[JOINT_NUM], q0[JOINT_NUM];
    float R_tgt[3][3];
    float J[6][JOINT_NUM];
    int iter;
    int use_ori = (rpy_deg != 0);

    if (cfg == 0 || target_xyz == 0 || q_out == 0) return 0;

    /* 目标旋转矩阵（若要求姿态） */
    if (use_ori) {
        rpy_to_rotation(rpy_deg[0], rpy_deg[1], rpy_deg[2], R_tgt);
    } else {
        /* 不约束姿态时，以当前位姿朝向为目标，使姿态误差项为 0 */
        memcpy(q0, q_out, sizeof(q0));
        float T0[4][4];
        fk(q0, T0);
        get_rotation(T0, R_tgt);
    }

    /* 初始解：从 q_out 中的当前关节角开始迭代 */
    memcpy(q, q_out, sizeof(q));

    for (iter = 0; iter < cfg->max_iter; iter++) {
        float T[4][4], p[3];
        float R[3][3];
        float e_pos[3] = {0,0,0}, e_ori[3] = {0,0,0};
        int n = 0, i, k;
        float Js[6][6];      /* 仅取约束行的雅可比（n x 6） */
        float A[6][6], b[6], dx[6], dq[JOINT_NUM];

        fk(q, T);
        get_position(T, p);

        /* 位置误差 */
        e_pos[0] = target_xyz[0] - p[0];
        e_pos[1] = target_xyz[1] - p[1];
        e_pos[2] = target_xyz[2] - p[2];

        /* 姿态误差 */
        get_rotation(T, R);
        orientation_error(R, R_tgt, e_ori);

        /* 收敛判断 */
        float pos_err = sqrtf(e_pos[0]*e_pos[0] + e_pos[1]*e_pos[1] + e_pos[2]*e_pos[2]);
        float ori_err = (use_ori) ? sqrtf(e_ori[0]*e_ori[0] + e_ori[1]*e_ori[1] + e_ori[2]*e_ori[2]) : 0.0f;
        if (pos_err < cfg->pos_tol &&
            (!use_ori || ori_err < cfg->ori_tol)) {
            memcpy(q_out, q, sizeof(q));
            return 1;
        }

        /* 雅可比 */
        numeric_jacobian(q, R_tgt, J);

        /* 按约束掩码组装缩减问题 */
        n = 0;
        for (i = 0; i < 6; i++) {
            if (cfg->mask[i]) {
                for (k = 0; k < JOINT_NUM; k++) Js[n][k] = J[i][k];
                if (i < 3) b[n] = e_pos[i]; else b[n] = e_ori[i - 3];
                n++;
            }
        }
        if (n == 0) return 0;

        /* A = Js·Js^T + λ²I */
        for (i = 0; i < n; i++) {
            for (k = 0; k < n; k++) {
                float s = 0.0f;
                for (int m = 0; m < JOINT_NUM; m++) s += Js[i][m] * Js[k][m];
                A[i][k] = s;
            }
            A[i][i] += cfg->lambda * cfg->lambda;
        }

        /* 解 (Js·Js^T + λ²I)·dx = b */
        memcpy(dx, b, sizeof(dx));
        if (!gauss_solve(n, A, dx)) {
            memcpy(q_out, q, sizeof(q));
            return 0;
        }

        /* dq = Js^T·dx */
        for (k = 0; k < JOINT_NUM; k++) {
            float s = 0.0f;
            for (i = 0; i < n; i++) s += Js[i][k] * dx[i];
            if (s > MAX_STEP)  s = MAX_STEP;
            if (s < -MAX_STEP) s = -MAX_STEP;
            dq[k] = s;
        }

        /* 更新关节角并限幅到关节限位 */
        for (k = 0; k < JOINT_NUM; k++) {
            q[k] += dq[k];
            if (q[k] < g_limits[k][0] * DEG2RAD) q[k] = g_limits[k][0] * DEG2RAD;
            if (q[k] > g_limits[k][1] * DEG2RAD) q[k] = g_limits[k][1] * DEG2RAD;
        }
    }

    memcpy(q_out, q, sizeof(q));
    return 0;   /* 达到最大迭代次数仍未收敛 */
}

int IK_SolvePosition(const float xyz[3], float q_out[JOINT_NUM])
{
    IK_Config_t cfg;
    IK_DefaultConfig(&cfg);
    /* 仅约束位置 */
    cfg.mask[3] = cfg.mask[4] = cfg.mask[5] = 0;
    return IK_Solve(&cfg, xyz, 0, q_out);
}

int IK_SolveFull(const float xyz[3], const float rpy_deg[3], float q_out[JOINT_NUM])
{
    IK_Config_t cfg;
    IK_DefaultConfig(&cfg);
    return IK_Solve(&cfg, xyz, rpy_deg, q_out);
}

void IK_ForwardKinematics(const float q_rad[JOINT_NUM], float xyz_out[3])
{
    float T[4][4];
    fk(q_rad, T);
    get_position(T, xyz_out);
}