/**
 * @file    pca9685_cascade.h
 * @brief   PCA9685 级联（双块）驱动
 * @note    第一阶段 1.2 硬件驱动层
 *
 * 两片 PCA9685 挂在同一 I2C 总线上，靠不同从机地址区分（级联）。
 * 本模块屏蔽"哪块板、哪个通道"的细节，对外提供统一的
 * 关节编号接口，最多支持 32 路通道。
 */
#ifndef __PCA9685_CASCADE_H
#define __PCA9685_CASCADE_H

#include "pca9685.h"
#include "config.h"

/* 级联板数量（本项目两块） */
#define CASCADE_BOARD_NUM   2u

/** 级联管理器句柄 */
typedef struct {
    PCA9685_Dev_t boards[CASCADE_BOARD_NUM];   /* 每块板的设备句柄 */
    uint8_t       count;                        /* 已挂载板数 */
} PCA9685_Cascade_t;

/** 板号枚举（1=板1, 2=板2） */
typedef enum {
    BOARD_1 = 1,
    BOARD_2 = 2,
} PCA9685_BoardId_t;

/* ==================== 对外接口 ==================== */

/** 初始化两块级联板（板1=0x40, 板2=0x41，见 config.h） */
void    PCA9685_Cascade_Init(PCA9685_Cascade_t *cas);

/** 设置指定板、指定通道的脉宽（ms） */
void    PCA9685_Cascade_SetPulseMs(PCA9685_Cascade_t *cas,
                                   PCA9685_BoardId_t board,
                                   uint8_t ch, float pulse_ms);

/** 按"关节号(1~6)"直接设置脉宽（自动路由到正确板与通道） */
void    PCA9685_Cascade_SetJointPulseMs(PCA9685_Cascade_t *cas,
                                        uint8_t joint, float pulse_ms);

/** 查询某关节实际对应的板号与通道（可打印调试） */
uint8_t PCA9685_Cascade_GetJointBoard(uint8_t joint);
uint8_t PCA9685_Cascade_GetJointChannel(uint8_t joint);

#endif /* __PCA9685_CASCADE_H */