/**
 * @file    adc.h
 * @brief   ADC 采集集中模块 — 轮速传感器诊断(6) + 压力(2) + VPOWER(1) + P12.1 模拟量
 *
 * @details 硬件资源 (两个 SAR 实例):
 *          ① SAR0 (PASS0_SAR0) — **9 通道合成一个 Group 自动序列**, 组头 CH0 软件触发,
 *             组尾 CH8 (grpDone)。每 10ms 触发一次读回全部 9 通道:
 *               CH0 P6.4 / CH1 P6.5 / CH2 P7.0 / CH3 P7.1 / CH4 P7.3 / CH5 P7.4
 *                 → 轮速传感器诊断 (开路/短路/间隙), 100 点环形缓冲 (1s 窗口)
 *               CH6 AN13 P7.5 前桥压力 | CH7 AN1 P6.1 后桥压力 (0.5~4.7V → 0~1050kPa)
 *               CH8 AN10 P7.2 VPOWER 分压 (47k+4.7k, 比 11.0, 0~55V)
 *          ② SAR1 (PASS0_SAR1) — P12.1 (AN5) 单通道, 独立实例/独立分频器,
 *             由 hw_rev.h 的 HW_REV_P12_1_ADC 开关 (默认 0 = 空实现)
 *
 * @note    调用顺序 (PSW_10ms_deal 内, **不能打乱**):
 *            Adc_Init()   : board_init() 中调用一次 (原 SensorDiag_Init/Sar1Adc_Init/P12_1_Adc_Init 合并)
 *            Adc_Read()   : 10ms 早期 (②), 触发并读回 SAR0 全通道 + P12.1
 *            Adc_Process(): 轮速计算之后 (⑥), 间隙诊断用当前车速做门控
 *          轮速采集本身 (TCPWM+PDMA) 在 wheel_speed.c, 不在本模块。
 */

#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stdint.h>
#include "peri_div.h"
#include "hw_rev.h"

/* ========================================================================== */
/*  SAR0 配置                                                                  */
/* ========================================================================== */
#define ADC_SAR0_MACRO          PASS0_SAR0
#define ADC_SAR0_PCLK           PCLK_PASS0_CLOCK_SAR0
#define ADC_SAR0_CLK_DIV        DIV16_NO_ADC
#define ADC_SAR0_MAX_FREQ_HZ    26670000ul   /* SAR 时钟上限 26.67MHz */

/* 通道分配: CH0~CH5 轮速诊断, CH6~CH8 压力/VPOWER (CH8 = 组尾) */
#define ADC_CH_WSS_NUM          6u
#define ADC_CH_PRESS_FRONT      6u
#define ADC_CH_PRESS_REAR       7u
#define ADC_CH_VPWR             8u
#define ADC_CH_TOTAL            9u
#define ADC_GROUP_HEAD_CH       0u           /* 软件触发此通道启动整个 group */

/* 通道使能开关 (诊断通道) */
#define ADC_DIAG_CH0_ENABLE     1u
#define ADC_DIAG_CH1_ENABLE     1u
#define ADC_DIAG_CH2_ENABLE     1u
#define ADC_DIAG_CH3_ENABLE     1u
#define ADC_DIAG_CH4_ENABLE     1u
#define ADC_DIAG_CH5_ENABLE     1u

#define IS_ADC_DIAG_CH_ENABLED(ch)  \
    ((ch) == 0u ? ADC_DIAG_CH0_ENABLE : \
     (ch) == 1u ? ADC_DIAG_CH1_ENABLE : \
     (ch) == 2u ? ADC_DIAG_CH2_ENABLE : \
     (ch) == 3u ? ADC_DIAG_CH3_ENABLE : \
     (ch) == 4u ? ADC_DIAG_CH4_ENABLE : \
     (ch) == 5u ? ADC_DIAG_CH5_ENABLE : 0u)

#define ADC_DIAG_BUF_SIZE       100u     /* 轮速诊断环形缓冲 (100 × 10ms = 1s) */

/* 轮询组尾 grpDone 的最大循环次数 (防 SAR 异常时死等, 20k × ~0.1μs ≈ 2ms) */
#define ADC_POLL_TIMEOUT_LOOP   20000u

/* ---- 引脚: SARMUX 直连 (HSIOM=0, ANALOG) ---- */
#define ADC_DIAG_CH0_PORT       GPIO_PRT6
#define ADC_DIAG_CH0_PIN        4u
#define ADC_DIAG_CH0_MUX        P6_4_GPIO
#define ADC_DIAG_CH0_AN         CY_ADC_PIN_ADDRESS_AN4

#define ADC_DIAG_CH1_PORT       GPIO_PRT6
#define ADC_DIAG_CH1_PIN        5u
#define ADC_DIAG_CH1_MUX        P6_5_GPIO
#define ADC_DIAG_CH1_AN         CY_ADC_PIN_ADDRESS_AN5

#define ADC_DIAG_CH2_PORT       GPIO_PRT7
#define ADC_DIAG_CH2_PIN        0u
#define ADC_DIAG_CH2_MUX        P7_0_GPIO
#define ADC_DIAG_CH2_AN         CY_ADC_PIN_ADDRESS_AN8

#define ADC_DIAG_CH3_PORT       GPIO_PRT7
#define ADC_DIAG_CH3_PIN        1u
#define ADC_DIAG_CH3_MUX        P7_1_GPIO
#define ADC_DIAG_CH3_AN         CY_ADC_PIN_ADDRESS_AN9

#define ADC_DIAG_CH4_PORT       GPIO_PRT7
#define ADC_DIAG_CH4_PIN        3u
#define ADC_DIAG_CH4_MUX        P7_3_GPIO
#define ADC_DIAG_CH4_AN         CY_ADC_PIN_ADDRESS_AN11

#define ADC_DIAG_CH5_PORT       GPIO_PRT7
#define ADC_DIAG_CH5_PIN        4u
#define ADC_DIAG_CH5_MUX        P7_4_GPIO
#define ADC_DIAG_CH5_AN         CY_ADC_PIN_ADDRESS_AN12

#define ADC_CH_FRONT_PORT       GPIO_PRT7     /* 前桥压力 CH6 */
#define ADC_CH_FRONT_PIN        5u
#define ADC_CH_FRONT_MUX        P7_5_GPIO
#define ADC_CH_FRONT_AN         CY_ADC_PIN_ADDRESS_AN13

#define ADC_CH_REAR_PORT        GPIO_PRT6     /* 后桥压力 CH7 */
#define ADC_CH_REAR_PIN         1u
#define ADC_CH_REAR_MUX         P6_1_GPIO
#define ADC_CH_REAR_AN          CY_ADC_PIN_ADDRESS_AN1

#define ADC_CH_VPWR_PORT        GPIO_PRT7     /* VPOWER CH8 */
#define ADC_CH_VPWR_PIN         2u
#define ADC_CH_VPWR_MUX         P7_2_GPIO
#define ADC_CH_VPWR_AN          CY_ADC_PIN_ADDRESS_AN10

/* ---- 电压换算常量 ---- */
#define ADC_VREF_MV             5000u
#define ADC_12BIT_MAX           4095u
#define ADC_MV_TO_RAW(mv)       ((mv) * ADC_12BIT_MAX / ADC_VREF_MV)

/* ---- 轮速传感器故障阈值 (12bit raw) ---- */
#define DIAG_OPEN_MIN           0x1C0
#define DIAG_OPEN_MAX           0x22D
#define DIAG_OPEN_HYST          0x50

#define DIAG_SHORT_MIN          0x9B0
#define DIAG_SHORT_MAX          0xAEF
#define DIAG_SHORT_HYST         0x50

#define DIAG_GAP_AMPL_MIN       0x30
#define DIAG_GAP_AMPL_HYST      0x10
#define DIAG_GAP_RPM_THRESH     417u   /* 15km/h ≈ 417mm/s, 与 wheel_speed 单位对齐 */

/* ---- 压力传感器故障诊断 (双层窗口 + 对称消抖) ----
 *  0V ── 0.23V ── 0.5V ──────────── 4.7V ── 4.77V ── 5V
 *        ├故障区┤├容忍┤├ 正常计算区 ┤├容忍┤├故障区┤
 *              ERR_MIN NORMAL_MIN NORMAL_MAX ERR_MAX                        */
#define PRESS_ERR_MIN_RAW       189u    /* 0.23V */
#define PRESS_ERR_MAX_RAW       3907u   /* 4.77V */
#define PRESS_NORMAL_MIN_RAW    410u    /* 0.5V  */
#define PRESS_NORMAL_MAX_RAW    3849u   /* 4.7V  */
#define PRESS_FAULT_STREAK      50u     /* 0.5s 连报 / 0.5s 连恢复 */
#define PRESS_MAX_KPA           1050u   /* 最大量程 */

/* ---- VPOWER 分压 (R1=47k, R2=4.7k) + 报警阈值 ---- */
#define VPWR_DIVIDER_RATIO_NUM  110u    /* 11.0 × 10 */
#define VPWR_DIVIDER_RATIO_DEN   10u
#define VPWR_ALARM_HIGH_MV      360u    /* 36.0V (fact 0.1V) */
#define VPWR_ALARM_LOW_MV       180u    /* 18.0V (fact 0.1V) */
#define VPWR_ALARM_STREAK        10u    /* 对称消抖次数 */

/* ========================================================================== */
/*  P12.1 独立 SAR1 (HW_REV_P12_1_ADC 开关)                                     */
/* ========================================================================== */
#if (HW_REV_P12_1_ADC == 1u)
#define P12_1_ADC_MACRO         PASS0_SAR1
#define P12_1_ADC_PCLK          PCLK_PASS0_CLOCK_SAR1
#define P12_1_ADC_CLK_DIV       DIV16_NO_ADC2   /* 独立分频器, 与 SAR0 分开 */
#define P12_1_ADC_CH            0u              /* 单通道, 组头=组尾 */
#define P12_1_ADC_PORT          GPIO_PRT12
#define P12_1_ADC_PIN           1u
#define P12_1_ADC_MUX           P12_1_GPIO
#define P12_1_ADC_VREF_MV       5000u
#define P12_1_ADC_12BIT_MAX     4095u
#endif

/* ========================================================================== */
/*  数据结构                                                                   */
/* ========================================================================== */

/** @brief 单通道轮速传感器诊断结果 */
typedef struct
{
    uint16_t adc_raw;
    uint16_t voltage_mv;
    bool     fault_short;
    bool     fault_open;
    bool     fault_gap_too_large;
} sensor_ch_diag_t;

/** @brief 六通道故障汇总 (供 psw_data 取用) */
typedef struct
{
    bool ch0_fault_open;
    bool ch0_fault_short;
    bool ch0_fault_gap_too_large;
    bool ch1_fault_open;
    bool ch1_fault_short;
    bool ch1_fault_gap_too_large;
    bool ch2_fault_open;
    bool ch2_fault_short;
    bool ch2_fault_gap_too_large;
    bool ch3_fault_open;
    bool ch3_fault_short;
    bool ch3_fault_gap_too_large;
    bool ch4_fault_open;
    bool ch4_fault_short;
    bool ch4_fault_gap_too_large;
    bool ch5_fault_open;
    bool ch5_fault_short;
    bool ch5_fault_gap_too_large;
} sensor_fault_status_t;

extern volatile sensor_ch_diag_t     g_sensor_diag[ADC_CH_WSS_NUM];
extern volatile sensor_fault_status_t g_sensor_fault_status;

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

/** @brief ADC 初始化: SAR0 (时钟/引脚/9 通道) + P12.1 (独立 SAR1) — board_init() 调用一次 */
void Adc_Init(void);

/** @brief 触发一轮采集并读回: SAR0 group (CH0~CH8) + P12.1; 10ms 早期调用 (须早于 Adc_Process) */
void Adc_Read(void);

/** @brief 轮速诊断统计与判故障 (需 100 点缓冲满; 用当前车速做间隙门控) — 轮速计算之后调用 */
void Adc_Process(void);

/** @brief ADC 组转换超时次数 (SAR 异常诊断用, 正常恒 0) */
uint16_t Adc_GetTimeoutCount(void);

/* ---- 压力查询 (前桥 CH6 / 后桥 CH7) ---- */
uint16_t Pressure_GetFrontRaw(void);
uint16_t Pressure_GetRearRaw(void);
uint16_t Pressure_GetFrontKpa(void);
uint16_t Pressure_GetRearKpa(void);
bool     Pressure_IsFrontFault(void);
bool     Pressure_IsRearFault(void);

/* ---- VPOWER 查询 (CH8, 继电器后) ---- */
uint16_t Vpower_GetRaw(void);         /* ADC 原始值 */
uint16_t Vpower_GetVoltage(void);     /* fact:0.1V */
bool     Vpower_IsHighAlarm(void);    /* > 36.0V */
bool     Vpower_IsLowAlarm(void);     /* < 18.0V */

/* ---- P12.1 查询 (HW_REV_P12_1_ADC=0 时恒 0) ---- */
uint16_t P12_1_GetRaw(void);
uint16_t P12_1_GetVoltageMv(void);

#endif /* ADC_H */
