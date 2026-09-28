/**
 * @file    wheel_speed.h
 * @brief   六通道轮速采集 — TCPWM 捕获 + PDMA 搬运 + 时间差法测速
 *
 * @details - TCPWM 捕获 CC0(上升沿)/CC1(下降沿), CC1_MATCH 经 Trigger-MUX 3
 *            触发 PDMA, 硬件把 CC0/CC1 搬进环形缓冲, CPU 零开销
 *          - 每 10ms 主循环取走新增脉冲, 末脉冲时间差法测速 + 脉宽过滤
 *            + 低速插值, 输出 mm/s
 *
 * @note    时钟链: PERI_CLK 40MHz → DIV16_NO_WHEEL(2MHz) → TCPWM ÷2 → 1MHz
 *          (1 tick = 1us; 计数器 period = 60000 → 60ms 环绕)
 *          Trigger-MUX: TCPWM tr_out0 → Input Group 3 → Output Group 3 (PDMA0)
 *          → dw0_tr_in[n], 通道号一一对应, 故 6 通道占 DW0 CH8~CH13
 */

#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdbool.h>
#include <stdint.h>
#include "hw_rev.h"
#include "peri_div.h"

/* ========================================================================== */
/*  公共参数                                                                   */
/* ========================================================================== */
#define NUM_CAPTURE_CHANNELS    6u       /* 轮速通道数 */

/* 捕获时钟: 分频器输出 2MHz, TCPWM 再 ÷2 → 1MHz (1us/tick) */
#define CAPTURE_CLK_TARGET_HZ   2000000ul

/* 每 10ms 单通道最大捕获脉冲数 (决定 DMA 环形缓冲深度) */
#define TRIG_MAX_COUNT_PER_10MS 30u

/* 捕获计数器参数 (1MHz → period 60000 = 60ms) */
#define PWM_CAP_PERIOD_TICKS    60000u
#define PWM_CAP_PRESCALER       CY_TCPWM_PRESCALER_DIVBY_2

/* 脉宽下限 (1us 单位): 窄于此值的脉冲判为毛刺丢弃
 * 实际值 = 60000/480 = 125us (宏名含 250US 但不反映实际值, 以本式为准) */
#define PWM_CAP_250US           (PWM_CAP_PERIOD_TICKS / 480u)   /* = 125 */

/* ========================================================================== */
/*  六路捕获通道硬件资源 — 引脚沿用当前硬件 (CH0 ~ CH5)                          */
/*                                                                            */
/*  CH0: P8.2  TR_ONE_CNT_IN63  TCPWM0_GRP0_CNT21 — XL (附加桥左)              */
/*  CH1: P12.0 TR_ONE_CNT_IN108 TCPWM0_GRP0_CNT36 — FL (前桥左)                */
/*  CH2: P8.1  TR_ONE_CNT_IN60  TCPWM0_GRP0_CNT20 — XR (附加桥右, 飞线后)      */
/*  CH3: P12.2 TR_ONE_CNT_IN114 TCPWM0_GRP0_CNT38 — RR (后桥右)                */
/*  CH4: P12.3 TR_ONE_CNT_IN117 TCPWM0_GRP0_CNT39 — RL (后桥左)                */
/*  CH5: P12.4 TR_ONE_CNT_IN120 TCPWM0_GRP0_CNT40 — FR (前桥右)                */
/* ========================================================================== */

/* CH0: P8.2, TR_ONE_CNT_IN63, TCPWM0_GRP0_CNT21 */
#define CH0_TCPWM              TCPWM0_GRP0_CNT21
#define CH0_PCLK               PCLK_TCPWM0_CLOCKS21
#define CH0_PORT               GPIO_PRT8
#define CH0_PIN                2ul
#define CH0_MUX                P8_2_TCPWM0_TR_ONE_CNT_IN63
#define CH0_DMA_TRGI           TRIG_IN_MUX_3_TCPWM_16_TR_OUT021
#define CH0_DMA_TRGO           TRIG_OUT_MUX_3_PDMA0_TR_IN8
#define CH0_DMA_CHN            8u
#define CH0_CLK_DIV            DIV16_NO_WHEEL

/* CH1: P12.0, TR_ONE_CNT_IN108, TCPWM0_GRP0_CNT36 */
#define CH1_TCPWM              TCPWM0_GRP0_CNT36
#define CH1_PCLK               PCLK_TCPWM0_CLOCKS36
#define CH1_PORT               GPIO_PRT12
#define CH1_PIN                0ul
#define CH1_MUX                P12_0_TCPWM0_TR_ONE_CNT_IN108
#define CH1_DMA_TRGI           TRIG_IN_MUX_3_TCPWM_16_TR_OUT036
#define CH1_DMA_TRGO           TRIG_OUT_MUX_3_PDMA0_TR_IN9
#define CH1_DMA_CHN            9u
#define CH1_CLK_DIV            DIV16_NO_WHEEL

/* CH2: P8.1, TR_ONE_CNT_IN60, TCPWM0_GRP0_CNT20 (飞线后固定, 见 hw_rev.h) */
#define CH2_TCPWM              TCPWM0_GRP0_CNT20
#define CH2_PCLK               PCLK_TCPWM0_CLOCKS20
#define CH2_PORT               GPIO_PRT8
#define CH2_PIN                1ul
#define CH2_MUX                P8_1_TCPWM0_TR_ONE_CNT_IN60
#define CH2_DMA_TRGI           TRIG_IN_MUX_3_TCPWM_16_TR_OUT020
#define CH2_DMA_TRGO           TRIG_OUT_MUX_3_PDMA0_TR_IN10
#define CH2_DMA_CHN            10u
#define CH2_CLK_DIV            DIV16_NO_WHEEL

/* CH3: P12.2, TR_ONE_CNT_IN114, TCPWM0_GRP0_CNT38 */
#define CH3_TCPWM              TCPWM0_GRP0_CNT38
#define CH3_PCLK               PCLK_TCPWM0_CLOCKS38
#define CH3_PORT               GPIO_PRT12
#define CH3_PIN                2ul
#define CH3_MUX                P12_2_TCPWM0_TR_ONE_CNT_IN114
#define CH3_DMA_TRGI           TRIG_IN_MUX_3_TCPWM_16_TR_OUT038
#define CH3_DMA_TRGO           TRIG_OUT_MUX_3_PDMA0_TR_IN11
#define CH3_DMA_CHN            11u
#define CH3_CLK_DIV            DIV16_NO_WHEEL

/* CH4: P12.3, TR_ONE_CNT_IN117, TCPWM0_GRP0_CNT39 */
#define CH4_TCPWM              TCPWM0_GRP0_CNT39
#define CH4_PCLK               PCLK_TCPWM0_CLOCKS39
#define CH4_PORT               GPIO_PRT12
#define CH4_PIN                3ul
#define CH4_MUX                P12_3_TCPWM0_TR_ONE_CNT_IN117
#define CH4_DMA_TRGI           TRIG_IN_MUX_3_TCPWM_16_TR_OUT039
#define CH4_DMA_TRGO           TRIG_OUT_MUX_3_PDMA0_TR_IN12
#define CH4_DMA_CHN            12u
#define CH4_CLK_DIV            DIV16_NO_WHEEL

/* CH5: P12.4, TR_ONE_CNT_IN120, TCPWM0_GRP0_CNT40 */
#define CH5_TCPWM              TCPWM0_GRP0_CNT40
#define CH5_PCLK               PCLK_TCPWM0_CLOCKS40
#define CH5_PORT               GPIO_PRT12
#define CH5_PIN                4ul
#define CH5_MUX                P12_4_TCPWM0_TR_ONE_CNT_IN120
#define CH5_DMA_TRGI           TRIG_IN_MUX_3_TCPWM_16_TR_OUT040
#define CH5_DMA_TRGO           TRIG_OUT_MUX_3_PDMA0_TR_IN13
#define CH5_DMA_CHN            13u
#define CH5_CLK_DIV            DIV16_NO_WHEEL

/* ========================================================================== */
/*  轮速换算参数                                                               */
/* ========================================================================== */
#define GEAR_DIAMETER_MM       110u
#define GEAR_TEETH_NUM         100u

/* ========================================================================== */
/*  全局变量                                                                   */
/* ========================================================================== */

/* 滤波后轮速 (单位 mm/s), 由 WheelSpeed_CalcAndSend 填充, psw_data 读取 */
extern uint16_t g_wheel_speed_rpm[NUM_CAPTURE_CHANNELS];

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

/**
 * @brief   初始化六路 TCPWM 捕获通道 (时钟 + GPIO + PDMA + Trigger-MUX)
 */
void WheelSpeed_Init(void);

/**
 * @brief   轮速计算 — 主循环每 10ms 调用
 *
 * @details 从 PDMA 环形缓冲取出本周期新增脉冲, 时间差法测速,
 *          250us 脉宽过滤后写入 g_wheel_speed_rpm[] (mm/s)。
 */
void WheelSpeed_CalcAndSend(void);

/**
 * @brief   整车参考车速: 六通道中值 (mm/s)
 * @note    用于传感器间隙诊断的速度门控, 避免单通道故障自我阻塞。
 */
uint16_t WheelSpeed_VehicleRefRpm(void);

#endif /* WHEEL_SPEED_H */
