/**
 * @file    pca9685.c
 * @brief   PCA9685 PWM 驱动板驱动（单块）实现
 * @note    第一阶段 1.1 硬件驱动层
 */
#include "pca9685.h"

/* 振荡器频率 25MHz，用于计算预分频 */
#define PCA9685_OSC_FREQ       25000000UL

/**
 * @brief  向 PCA9685 写一个字节寄存器
 * @return 1=成功 0=失败
 */
uint8_t PCA9685_WriteByte(PCA9685_Dev_t *dev, uint8_t reg, uint8_t val)
{
    if (dev == 0 || dev->hi2c == 0) {
        return 0;
    }
    return (HAL_I2C_Mem_Write(dev->hi2c, dev->addr, reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK);
}

/**
 * @brief  从 PCA9685 读一个字节寄存器
 * @return 1=成功 0=失败
 */
uint8_t PCA9685_ReadByte(PCA9685_Dev_t *dev, uint8_t reg, uint8_t *val)
{
    if (dev == 0 || dev->hi2c == 0 || val == 0) {
        return 0;
    }
    return (HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg,
                             I2C_MEMADD_SIZE_8BIT, val, 1, 100) == HAL_OK);
}

/**
 * @brief  复位 PCA9685（回到默认寄存器值）
 */
void PCA9685_Reset(PCA9685_Dev_t *dev)
{
    if (dev == 0) return;
    /* 写入任意值触发复位，然后等待振荡器稳定 */
    PCA9685_WriteByte(dev, PCA9685_REG_MODE1, 0x00);
    HAL_Delay(10);
}

/**
 * @brief  初始化单块 PCA9685
 * @param  addr  板卡 I2C 地址（如 0x40）
 */
void PCA9685_Init(PCA9685_Dev_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    if (dev == 0) return;

    dev->hi2c    = hi2c;
    dev->addr    = addr;
    dev->freq    = PCA9685_PWM_FREQ_HZ;
    dev->init_ok = 0;

    PCA9685_Reset(dev);

    /* 开启 Auto-Increment，方便连续写 LED 寄存器 */
    uint8_t mode1 = PCA9685_MODE1_AI;
    if (!PCA9685_WriteByte(dev, PCA9685_REG_MODE1, mode1)) {
        return;                          /* I2C 不通，初始化失败 */
    }

    /* 默认输出推挽模式 */
    PCA9685_WriteByte(dev, PCA9685_REG_MODE2, 0x04);

    /* 设置 50Hz 输出频率 */
    PCA9685_SetPWMFreq(dev, PCA9685_PWM_FREQ_HZ);

    /* 关闭所有通道输出（占空比 0） */
    PCA9685_SetPWMCounts(dev, 0xFF, 0, 0);   /* 通道 0xFF 表示"所有通道" */
    dev->init_ok = 1;
}

/**
 * @brief  设置 PWM 输出频率（Hz）
 *
 *         预分频计算：prescale = 25000000 / (4096 * freq) - 1
 *         写入前必须进入 SLEEP 模式，写后再退出。
 */
void PCA9685_SetPWMFreq(PCA9685_Dev_t *dev, uint16_t freq)
{
    uint8_t prescale;

    if (dev == 0 || freq == 0) return;
    dev->freq = freq;

    /* 计算并取整预分频值 */
    float p = (float)PCA9685_OSC_FREQ / (4096UL * freq) - 1.0f;
    if (p < 3.0f)  p = 3.0f;
    if (p > 255.0f) p = 255.0f;
    prescale = (uint8_t)(p + 0.5f);        /* 四舍五入 */

    uint8_t mode1;
    PCA9685_ReadByte(dev, PCA9685_REG_MODE1, &mode1);

    /* 进入 SLEEP：必须的，否则无法改预分频 */
    PCA9685_WriteByte(dev, PCA9685_REG_MODE1, (mode1 & 0x7F) | PCA9685_MODE1_SLEEP);
    PCA9685_WriteByte(dev, PCA9685_REG_PRESCALE, prescale);

    /* 唤醒（清除 SLEEP）并稍作延时 */
    PCA9685_WriteByte(dev, PCA9685_REG_MODE1, mode1 & 0x7F);
    HAL_Delay(5);

    /* 重启以应用新频率 */
    PCA9685_WriteByte(dev, PCA9685_REG_MODE1, (mode1 & 0x7F) | PCA9685_MODE1_RESTART);
}

/**
 * @brief  设置通道占空比（12 位计数值）
 * @param  ch  通道号 0~15，或 0xFF 表示所有通道
 * @param  on  ON 边沿计数值（0~4095）
 * @param  off OFF 边沿计数值（0~4095），占空比 = (off-on)/4096
 *
 * 例：50Hz 下 1ms 脉宽 = 4096 * 1/20 ≈ 205 计数值。
 *     设 on=0, off=205 即可。
 */
void PCA9685_SetPWMCounts(PCA9685_Dev_t *dev, uint8_t ch, uint16_t on, uint16_t off)
{
    uint8_t reg;

    if (dev == 0 || ch > 0xFF) return;

    reg = (ch == 0xFF) ? PCA9685_REG_ALL_LED_ON_L
                       : PCA9685_REG_LED0_ON_L + 4u * ch;

    uint8_t buf[4];
    buf[0] = (uint8_t)(on  & 0xFF);
    buf[1] = (uint8_t)(on  >> 8);
    buf[2] = (uint8_t)(off & 0xFF);
    buf[3] = (uint8_t)(off >> 8);

    HAL_I2C_Mem_Write(dev->hi2c, dev->addr, reg,
                      I2C_MEMADD_SIZE_8BIT, buf, 4, 100);
}

/**
 * @brief  设置通道脉宽（ms）
 * @param  ch 通道号 0~15
 * @param  pulse_ms 脉宽，单位毫秒。50Hz 下范围约 0.5~2.5ms
 *
 * 计数值 = 脉宽(ms) / 周期(ms) * 4096
 */
void PCA9685_SetPulseMs(PCA9685_Dev_t *dev, uint8_t ch, float pulse_ms)
{
    float period_ms = 1000.0f / (float)dev->freq;
    uint32_t counts;

    if (pulse_ms <= 0.0f) {
        counts = 0u;                    /* 占空比为 0，舵机停转 */
    } else if (pulse_ms >= period_ms) {
        counts = 4096u;                 /* 满周期（全高） */
    } else {
        counts = (uint32_t)((pulse_ms / period_ms) * 4096.0f + 0.5f);
    }

    if (counts > 4095u) counts = 4095u;
    PCA9685_SetPWMCounts(dev, ch, 0u, (uint16_t)counts);
}