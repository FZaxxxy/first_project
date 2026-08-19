/**
 * @file    vision.c
 * @brief   目标识别与坐标转换实现
 * @note    第三阶段 3.2 智能算法层
 *
 * 坐标转换模型（针孔模型投影到桌面平面 z=0）：
 *
 *   相机位于基座上方 CAM_POS，向前下方俯视 CAM_TILT 度。
 *   1. 像素 (u,v) 归一化到像平面坐标 (x_n, y_n)，利用焦距换算为射线方向；
 *   2. 射线经相机位姿旋转到基座坐标系；
 *   3. 与桌面平面 z=0 求交，得到目标在基座坐标系下的 (x, y)。
 *
 * 注意：符号/旋转方向可能随相机安装方式而异，
 *       上电后可用已知位置的物体实测校准（标定参数在 config.h）。
 */
#include "vision.h"
#include "sensor.h"
#include <math.h>

#define VISION_DATA_TIMEOUT_MS   500u    /* 超过此时间无新帧视为无目标 */

/* 相机内参（由视场角与分辨率推导，★ 可按实际镜头标定调整） */
static float g_fx;                       /* 焦距 x（像素） */
static float g_fy;                       /* 焦距 y（像素） */

/* 相机位姿旋转矩阵 R（列：相机 x/y/z 轴在基座系中的方向） */
static float g_R[3][3];

void Vision_Init(void)
{
    float fov_x_rad = CAM_FOV_X_DEG * 0.01745329252f;
    float fov_y_rad = fov_x_rad * (float)CAM_IMG_H / (float)CAM_IMG_W;

    g_fx = (float)CAM_IMG_W / 2.0f / tanf(fov_x_rad / 2.0f);
    g_fy = (float)CAM_IMG_H / 2.0f / tanf(fov_y_rad / 2.0f);

    /* 相机朝向：沿 +y 前进、向下俯视 CAM_TILT 度（θ 为与水平面夹角） */
    float th = CAM_TILT_DEG * 0.01745329252f;
    float st = sinf(th), ct = cosf(th);

    /* R 各列为相机右(x)/下(y)/前(z) 轴在基座系中的方向 */
    g_R[0][0] = 1.0f;  g_R[0][1] = 0.0f;      g_R[0][2] = 0.0f;
    g_R[1][0] = 0.0f;  g_R[1][1] = -st;       g_R[1][2] =  ct;
    g_R[2][0] = 0.0f;  g_R[2][1] = -ct;       g_R[2][2] = -st;
}

void Vision_PixelToBase(float px, float py, float *x, float *y)
{
    float xn = (px - (float)CAM_IMG_W / 2.0f) / g_fx;   /* 归一化像平面 x */
    float yn = (py - (float)CAM_IMG_H / 2.0f) / g_fy;   /* 归一化像平面 y（下为正） */

    /* 相机系中的射线方向（前方 z=1） */
    float dc[3] = { xn, yn, 1.0f };

    /* 旋转到基座系 */
    float dw[3];
    dw[0] = g_R[0][0] * dc[0] + g_R[0][1] * dc[1] + g_R[0][2] * dc[2];
    dw[1] = g_R[1][0] * dc[0] + g_R[1][1] * dc[1] + g_R[1][2] * dc[2];
    dw[2] = g_R[2][0] * dc[0] + g_R[2][1] * dc[1] + g_R[2][2] * dc[2];

    /* 射线：P = P_cam + t·dw，与桌面 z=0 求交 */
    if (dw[2] >= 0.0f) {
        /* 朝上，不应发生：返回相机正下方投影点作为兜底 */
        *x = CAM_POS_X_M;
        *y = CAM_POS_Y_M;
        return;
    }
    float t = CAM_POS_Z_M / (-dw[2]);

    *x = CAM_POS_X_M + t * dw[0];
    *y = CAM_POS_Y_M + t * dw[1];
}

uint8_t Vision_GetTarget(float *x, float *y, float *z)
{
    Sensor_Vision_t v;

    Sensor_GetVision(&v);

    /* 数据有效性：有目标 且 时间戳未过期 */
    if (!v.valid || (HAL_GetTick() - v.stamp) > VISION_DATA_TIMEOUT_MS) {
        return 0;
    }

    Vision_PixelToBase(v.px, v.py, x, y);
    *z = TARGET_GRASP_Z_M;          /* 抓取高度（桌面上方） */
    return 1;
}