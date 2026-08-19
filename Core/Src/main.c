/**
 * @file    main.c
 * @brief   主程序入口
 * @note    第四阶段 4.1 系统集成
 *
 * 【集成说明】
 * 本项目代码按 CubeMX 工程结构组织。下方 MX_* 外设初始化函数
 * 为 STM32F103C8T6 的完整参考实现；若你使用 CubeMX 生成工程，
 * 请以 CubeMX 生成的同名函数为准（删除本文件中的重复定义），
 * 仅保留 main()、业务回调与主循环即可。
 *
 * 外设接线（★ 按实际板卡核对）：
 *   I2C1 : PB6(SCL) / PB7(SDA) -> 两片 PCA9685
 *   USART1: PA9(TX) / PA10(RX) -> USB转串口
 *   TIM2 : 1ms 节拍 -> 驱动舵机平滑控制与按键扫描
 *   按键 : PA0(下拉, 按下接地)  见 config.h
 *   光电 : PB0                    见 config.h
 *   夹爪限位 : PB1                见 config.h
 */
#include "main.h"
#include "app.h"
#include "servo.h"
#include "ui.h"
#include "debug.h"

/* ==================== 外设句柄 ==================== */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim2;

/* 1ms 节拍计数：每满 MOTION_TICK_MS 执行一次运动/UI 刷新 */
static volatile uint32_t g_tick_1ms = 0;

/* ==================== 主函数 ==================== */
int main(void)
{
    /* ---- 复位外设（CubeMX 生成） ---- */
    HAL_Init();

    /* ---- 系统时钟：72MHz ---- */
    SystemClock_Config();

    /* ---- 外设初始化 ---- */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();

    /* ---- 业务初始化（各层状态机） ---- */
    App_Init();

    DBG_I("机械臂智能体启动，请发送 HELP 查看指令");

    /* ---- 主循环 ---- */
    while (1) {
        App_Task();          /* 状态机 + 自动模式规划推进 */
        HAL_Delay(MOTION_TICK_MS);
    }
}

/* ==================== 业务回调 ==================== */

/**
 * @brief  定时器周期中断回调（HAL 在 TIM2_IRQHandler 中调用）
 * @note   每 1ms 一次；每 MOTION_TICK_MS 刷新一次舵机与 UI
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        g_tick_1ms++;
        if ((g_tick_1ms % MOTION_TICK_MS) == 0) {
            Servo_Update();  /* 推进各关节平滑运动 */
            UI_Tick();       /* 按键扫描 */
        }
    }
}

/**
 * @brief  串口接收完成中断回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        UI_UART_RxChar(*UI_UART_GetRxBytePtr());
    }
}

/* ==================== CubeMX 外设初始化（参考实现） ==================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* 外部 8MHz 晶振 -> PLL x9 -> 72MHz
     * ★ 须自定义：若板卡无外部晶振(HSE)，请改用 HSI 方案
     *   (HSI 只能达到 64MHz: 8/2 * 16) */
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState            = RCC_HSE_ON;
    osc.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL          = RCC_PLL_MUL9;          /* 8MHz x 9 = 72MHz */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;             /* 36MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;             /* 72MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 使能 GPIO 时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA0：模式按键输入（内部上拉，按下接地） */
    gpio.Pin   = BTN_MODE_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(BTN_MODE_PORT, &gpio);

    /* PB0：光电传感器输入（内部上拉） */
    gpio.Pin   = SENSOR_PHOTO_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(SENSOR_PHOTO_PORT, &gpio);

    /* PB1：夹爪限位开关输入（内部上拉） */
    gpio.Pin   = SENSOR_GRIP_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(SENSOR_GRIP_PORT, &gpio);
}

void HAL_I2C1_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};

    (void)hi2c;
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB6=SCL, PB7=SDA，开漏复用 + 上拉 */
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull      = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    (void)huart;
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA9=TX(推挽复用), PA10=RX(浮空输入) */
    gpio.Pin   = GPIO_PIN_9;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin   = GPIO_PIN_10;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 使能 USART1 全局中断 */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = DBG_BAUDRATE;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_TIM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

void MX_TIM2_Init(void)
{
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 36 - 1;      /* 36MHz/36 = 1MHz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 1000 - 1;    /* 1MHz/1000 = 1kHz = 1ms */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_Base_Start_IT(&htim2);
}

/* ==================== 错误处理 ==================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* 卡死，便于调试器定位 */
    }
}