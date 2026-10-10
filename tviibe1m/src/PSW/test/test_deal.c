/**
 * @file    test_deal.c
 * @brief   测试/调试层实现 — 阀测试 + CAN 调试上报 + 节拍验证
 *
 * @details 本文件收纳原先散落在 cmn.c / psw_data.c 的测试与调试上报代码:
 *            - 单阀 PWM 波形循环       (TEST_VALVE_DRV_WAVE)
 *            - 阀保压阶梯标定          (TEST_VALVE_HOLD → valve_hold_test.c)
 *            - RTE 变量 CAN 打印 0x700 (TEST_RTE_CAN_PRINT)
 *            - PSW 变量 CAN 打印 0x710 (TEST_PSW_CAN_PRINT)
 *            - 压力/VPOWER 上报 0x730  (TEST_PRESSURE_CAN)
 *            - 10ms 节拍翻转 P22.0      (TEST_TICK_DEBUG)
 *            - 阀诊断明细输出          (bts724g.h 的 VALVE_DIAG_DEBUG, 非本层开关)
 *
 *          开关全在 test_cfg.h; 三个入口的调用位置约束见 test_deal.h。
 *          总开关 = 0 时本文件只提供三个空入口, 保证 cmn.c 的调用照常链接。
 */

#include "test_cfg.h"
#include "test_deal.h"

#if (TEST_DEAL_ENABLE != 0u)

#include "cmn.h"
#include "can.h"
#include "gpio.h"
#include "psw_data.h"
#include "bts724g.h"
#include "valve_pwm.h"
#include "RTE.h"
#include "rte_psw.h"

#if (TEST_VALVE_HOLD != 0u)
#include "valve_hold_test.h"
#endif

/* ========================================================================== */
/*  阀波形测试 — 21/22 口进出气 (验证通信层四个外包函数)                        */
/*                                                                            */
/*  阶段1 (2000ms): 进气阀 21IN/22IN = 50ms 周期 / 10ms 高 (PWM 方波)          */
/*                  出气阀 21OUT/22OUT = 满占空比 (恒高, 关住排气)             */
/*  阶段2 (2000ms): 四阀全部 htime=0 → 关闭 PWM 输出                          */
/*  两阶段周期性交替; 路径: InValActX3_7 / OutValActX3_10 / InValActX4_16 /    */
/*  OutValActX2_16 (外包) → Psw_InValAct/OutValAct → ValvePwm_SetDutyLive      */
/*  ⚠ 只在阶段切换后的第一拍下发一次, 避免每 10ms 重置 PWM counter            */
/* ========================================================================== */
#if (TEST_VALVE_DRV_WAVE != 0u)

#define VEL_WAVE_PHASE_MS     2000u   /* 单阶段持续时长 (ms)               */
#define VEL_WAVE_IN_PERIOD    500u    /* 进气阀周期: 500 x 0.1ms = 50ms    */
#define VEL_WAVE_IN_HTIME     100u    /* 进气阀高电平: 100 x 0.1ms = 10ms  */

static void valve_drv_test_single(void)
{
    static uint16_t tick  = 0u;
    static uint8_t  phase = 0u;      /* 0 = 阶段1, 1 = 阶段2 */

    tick++;
    if (tick >= (VEL_WAVE_PHASE_MS / 10u))
    {
        tick  = 0u;                  /* 下拍 tick=1 → 切换阶段并下发 */
        phase = (uint8_t)(phase ^ 1u);
    }

    if (tick != 1u) { return; }      /* 只在阶段首拍下发 (含上电首拍) */

    if (phase == 0u)
    {
        /* 阶段1: 进气 PWM (50ms/10ms), 出气压满 (恒高, 保住压力) */
        InValActX3_7 (VEL_WAVE_IN_PERIOD, VEL_WAVE_IN_HTIME);        /* 21IN  */
        InValActX4_16(VEL_WAVE_IN_PERIOD, VEL_WAVE_IN_HTIME);        /* 22IN  */
        OutValActX3_10(VEL_WAVE_IN_PERIOD, VEL_WAVE_IN_PERIOD);      /* 21OUT 满占空比 */
        OutValActX2_16(VEL_WAVE_IN_PERIOD, VEL_WAVE_IN_PERIOD);      /* 22OUT 满占空比 */
    }
    else
    {
        /* 阶段2: 四阀关闭 PWM 输出 (htime = 0 → 关阀) */
        InValActX3_7 (VEL_WAVE_IN_PERIOD, 0u);
        InValActX4_16(VEL_WAVE_IN_PERIOD, 0u);
        OutValActX3_10(VEL_WAVE_IN_PERIOD, 0u);
        OutValActX2_16(VEL_WAVE_IN_PERIOD, 0u);
    }
}

#endif /* TEST_VALVE_DRV_WAVE */

/* ========================================================================== */
/*  RTE 变量 CAN 打印监控 (TEST_RTE_CAN_PRINT)                                 */
/*                                                                           */
/*  用调试 CAN (CAN1, 无发送白名单) 周期打印 RTE 变量, 监控 PSWDataToRte()    */
/*  是否正常更新 RTE。                                                        */
/*                                                                           */
/*  7 帧轮转, 每 RTE_DBG_PERIOD_MS 发一帧, 7 帧一个完整周期:                  */
/*    帧0 (ID+0): RTEvIgn / RTEBrkPreX4_14 / RTEBrkPreX4_15                   */
/*    帧1 (ID+1): RTEWheelSpeedFL / FR / RL                                   */
/*    帧2 (ID+2): RTEWheelSpeedRR / XL / XR                                   */
/*    帧3 (ID+3): 轮速传感器故障 (Open/Short/Gap × 6 通道)                     */
/*    帧4 (ID+4): ABS 阀故障 FL/FR/RL/RR (进/排 × 开/短)                      */
/*    帧5 (ID+5): ABS 阀故障 XL/XR/Tr + 21口(X3_7/X3_10) + 22口(X4_16/X2_16)  */
/*    帧6 (ID+6): 压力/电源/EEprom/Relay/Chip/ESCM + EEPROM 状态               */
/* ========================================================================== */
#if (TEST_RTE_CAN_PRINT != 0u)

#define RTE_DBG_CAN_ID       0x700u     /* 调试报文基准 ID (11bit 标准帧) */
#define RTE_DBG_PERIOD_MS    100u       /* 单帧打印周期 (10ms 整数倍)     */
#define RTE_DBG_FRAME_CNT    7u         /* 帧总数 (0~6)                   */

/* 故障标志 → 单 bit 值 (位域/整型字段统一转 0/1) */
#define RTE_BIT(v)  ((uint8_t)((v) ? 1u : 0u))

static void rte_can_print_monitor(void)
{
    static uint16_t tick  = 0u;
    static uint8_t  frame = 0u;
    uint8_t data[8];

    /* 节拍计数, 每 RTE_DBG_PERIOD_MS 发一帧 */
    tick++;
    if (tick < (RTE_DBG_PERIOD_MS / 10u))
    {
        return;
    }
    tick = 0u;

    data[0] = 0xFFu; data[1] = 0xFFu; data[2] = 0xFFu; data[3] = 0xFFu;
    data[4] = 0xFFu; data[5] = 0xFFu; data[6] = 0xFFu; data[7] = 0xFFu;

    switch (frame)
    {
    case 0u:    /* 帧0: VPOWER + 前/后桥压力 */
        data[0] = (uint8_t)(RTEvIgn >> 8);
        data[1] = (uint8_t)(RTEvIgn & 0xFFu);
        data[2] = (uint8_t)(RTEBrkPreX4_14 >> 8);
        data[3] = (uint8_t)(RTEBrkPreX4_14 & 0xFFu);
        data[4] = (uint8_t)(RTEBrkPreX4_15 >> 8);
        data[5] = (uint8_t)(RTEBrkPreX4_15 & 0xFFu);
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 0u, data, 6u);
        break;

    case 1u:    /* 帧1: 轮速 FL/FR/RL */
        data[0] = (uint8_t)(RTEWheelSpeedFL >> 8);
        data[1] = (uint8_t)(RTEWheelSpeedFL & 0xFFu);
        data[2] = (uint8_t)(RTEWheelSpeedFR >> 8);
        data[3] = (uint8_t)(RTEWheelSpeedFR & 0xFFu);
        data[4] = (uint8_t)(RTEWheelSpeedRL >> 8);
        data[5] = (uint8_t)(RTEWheelSpeedRL & 0xFFu);
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 1u, data, 6u);
        break;

    case 2u:    /* 帧2: 轮速 RR/XL/XR */
        data[0] = (uint8_t)(RTEWheelSpeedRR >> 8);
        data[1] = (uint8_t)(RTEWheelSpeedRR & 0xFFu);
        data[2] = (uint8_t)(RTEWheelSpeedXL >> 8);
        data[3] = (uint8_t)(RTEWheelSpeedXL & 0xFFu);
        data[4] = (uint8_t)(RTEWheelSpeedXR >> 8);
        data[5] = (uint8_t)(RTEWheelSpeedXR & 0xFFu);
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 2u, data, 6u);
        break;

    case 3u:    /* 帧3: 轮速传感器故障 (bit0~5: FL/FR/RL/RR/XL/XR) */
        data[0] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenFL)  << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenFR)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenRL)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenRR)  << 3u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenXL)  << 4u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssOpenXR)  << 5u) );
        data[1] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrWssShortFL) << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortFR) << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortRL) << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortRR) << 3u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortXL) << 4u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssShortXR) << 5u) );
        data[2] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrWssGapFL)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapFR)   << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapRL)   << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapRR)   << 3u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapXL)   << 4u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrWssGapXR)   << 5u) );
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 3u, data, 3u);
        break;

    case 4u:    /* 帧4: ABS 阀故障 FL/FR/RL/RR (bit0~3: InOpen/InShort/OutOpen/OutShort) */
        data[0] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenFL)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortFL)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenFL)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortFL) << 3u) );
        data[1] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenFR)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortFR)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenFR)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortFR) << 3u) );
        data[2] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenRL)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortRL)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenRL)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortRL) << 3u) );
        data[3] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenRR)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortRR)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenRR)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortRR) << 3u) );
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 4u, data, 4u);
        break;

    case 5u:    /* 帧5: ABS 阀 XL/XR + 21口(X3_7/X3_10) + 22口(X4_16/X2_16) + 低边回读[4]P2.0 [5]P6.2 [6]DiagForbit */
        data[0] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenXL)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortXL)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenXL)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortXL) << 3u) );
        data[1] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrInValOpenXR)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrInValShortXR)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValOpenXR)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrOutValShortXR) << 3u) );
        data[2] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrX3_7InValveOpen)    << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrX3_7InValveShort)   << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrX3_10OutValveOpen)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrX3_10OutValveShort) << 3u) );
        data[3] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrX4_16InValveOpen)   << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrX4_16InValveShort)  << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrX2_16OutValveOpen)  << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrX2_16OutValveShort) << 3u) );
        /* 低边开关实时回读 (0=断开 1=闭合) + 阀诊断让路标志 */
        data[4] = (uint8_t)((InLowSideStX4_16()   != 0u) ? 1u : 0u);   /* P2.0 前 ASR 低边 */
        data[5] = (uint8_t)((OutLowSideStX2_16()  != 0u) ? 1u : 0u);   /* P6.2 后 ASR 低边 */
        data[6] = (uint8_t)((RTEfValWssTestForbit != 0u) ? 1u : 0u);   /* 1=阀诊断已停跑 */
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 5u, data, 7u);
        break;

    case 6u:    /* 帧6: 压力/电源/EEprom/Relay/Chip/ESCM + EEPROM 状态 */
        /* 压力故障是独立全局 RTEfErrPreX4_14/X4_15 → bit2/bit3 (bit0/bit1 保留) */
        data[0] = (uint8_t)( (RTE_BIT(RTEfErrPreX4_14)        << 2u)
                           | (RTE_BIT(RTEfErrPreX4_15)        << 3u) );
        data[1] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrVbatLo) << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrVbatHi) << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrVpwrLo) << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrVpwrHi) << 3u) );
        data[2] = (uint8_t)( (RTE_BIT(RTEfPSWErr.RTEfErrEEprom)       << 0u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrEEpromOutBnd) << 1u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrRelay)        << 2u)
                           | (RTE_BIT(RTEfPSWErr.RTEfErrDriveChip)    << 3u) );
        data[3] = (uint8_t)( (RTE_BIT(RTEfCOMErr.RTEfErrSASNoCalib) << 0u)
                           | (RTE_BIT(RTEfCOMErr.RTEfErrSASLost)    << 1u)
                           | (RTE_BIT(RTEfCOMErr.RTEfErrSASSgn)     << 2u) );
        data[4] = RTEfEESt;    /* EEPROM 状态: 255=首次写, 86=非首次写 */
        (void)can1_sendMsg(RTE_DBG_CAN_ID + 6u, data, 5u);
        break;

    default:
        frame = 0u;
        break;
    }

    frame++;
    if (frame >= RTE_DBG_FRAME_CNT)
    {
        frame = 0u;
    }
}

#endif /* TEST_RTE_CAN_PRINT */

/* ========================================================================== */
/*  PSW 变量 CAN 打印监控 (TEST_PSW_CAN_PRINT)                                 */
/*                                                                           */
/*  用调试 CAN (CAN1, 无发送白名单) 周期打印 PSW* 中间变量, 监控驱动层 →      */
/*  PSW* 是否正常更新。与 RTE 层打印 (0x700 起) 对比, 可定位 PSWDataToRte()   */
/*  转换/掩蔽问题。PSW 特有: 芯片级故障 / 压力原始 kpa / 轮速 0.1km/h 原始值   */
/*  / PSWTaskTime / resetReason。                                            */
/*                                                                           */
/*  19 帧轮转 (8/9/12~16/18 已裁剪不发送), 每 PSW_DBG_PERIOD_MS 发一帧:                  */
/*    帧0 (ID+0): PSWTaskTime / PSWvIgn / 前桥压力 / 后桥压力 (原始 kpa)      */
/*    帧1 (ID+1): PSWWheelSpeedFL / FR / RL (0.1km/h)                        */
/*    帧2 (ID+2): PSWWheelSpeedRR / XL / XR                                  */
/*    帧3 (ID+3): 轮速传感器故障 (Open/Short/Gap × 6 通道)                     */
/*    帧4 (ID+4): ABS 阀故障 FL/FR/RL/RR (进/排 × 开/短)                      */
/*    帧5 (ID+5): ABS 阀 XL/XR + 21口/22口 + TR_ASR(仅 PSW 内部)              */
/*    帧6 (ID+6): 驱动芯片故障 (byte0=Open, byte1=Short, bit0~4=U6/U9/U12/U13/U19) */
/*    帧7 (ID+7): 压力/电源/Relay/EEprom 故障 + 重启原因 resetReason          */
/*    帧10/11 (ID+10/11): 【S2】->ValvePwm 写入实参: period / htime (10us步, U16 大端) */
/*    帧17 (ID+17): 【S2】实算: 占空比%(4B) + 周期(x2ms,4B)                     */
/*    (帧8/9/12~16/18 已裁剪: S1/S3 采样点与计数帧已删除, 不再发送)              */
/* ========================================================================== */
#if (TEST_PSW_CAN_PRINT != 0u)

#define PSW_DBG_CAN_ID       0x710u     /* 调试报文基准 ID (11bit 标准帧) */
#define PSW_DBG_PERIOD_MS    100u       /* 单帧打印周期 (10ms 整数倍)     */
#define PSW_DBG_FRAME_CNT    19u         /* 帧总数 (0~18)                   */

/* 故障标志 → 单 bit 值 */
#define PSW_BIT(v)  ((uint8_t)((v) ? 1u : 0u))

/* 帧16/17 公共: 由 period/htime 算占空比% (clamp 100; period=0 视为 0%) */
static uint8_t dbg_duty_pct(uint16_t period, uint16_t htime)
{
    if (period == 0u) { return 0u; }
    const uint32_t d = ((uint32_t)htime * 100u + (uint32_t)(period / 2u)) / (uint32_t)period;
    return (uint8_t)((d > 100u) ? 100u : d);
}

static void psw_can_print_monitor(void)
{
    static uint16_t tick  = 0u;
    static uint8_t  frame = 0u;
    uint8_t data[8];

    /* 节拍计数, 每 PSW_DBG_PERIOD_MS 发一帧 */
    tick++;
    if (tick < (PSW_DBG_PERIOD_MS / 10u))
    {
        return;
    }
    tick = 0u;

    data[0] = 0xFFu; data[1] = 0xFFu; data[2] = 0xFFu; data[3] = 0xFFu;
    data[4] = 0xFFu; data[5] = 0xFFu; data[6] = 0xFFu; data[7] = 0xFFu;

    switch (frame)
    {
    case 0u:    /* 帧0: 任务时间 + VPOWER + 前/后桥压力 (原始 kpa) */
        data[0] = PSWTaskTime;
        data[1] = (uint8_t)(PSWvIgn >> 8);
        data[2] = (uint8_t)(PSWvIgn & 0xFFu);
        data[3] = (uint8_t)(PSWBrkPreX2_7 >> 8);
        data[4] = (uint8_t)(PSWBrkPreX2_7 & 0xFFu);
        data[5] = (uint8_t)(PSWBrkPreX3_10 >> 8);
        data[6] = (uint8_t)(PSWBrkPreX3_10 & 0xFFu);
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 0u, data, 7u);
        break;

    case 1u:    /* 帧1: 轮速 FL/FR/RL (0.1km/h) */
        data[0] = (uint8_t)(PSWWheelSpeedFL >> 8);
        data[1] = (uint8_t)(PSWWheelSpeedFL & 0xFFu);
        data[2] = (uint8_t)(PSWWheelSpeedFR >> 8);
        data[3] = (uint8_t)(PSWWheelSpeedFR & 0xFFu);
        data[4] = (uint8_t)(PSWWheelSpeedRL >> 8);
        data[5] = (uint8_t)(PSWWheelSpeedRL & 0xFFu);
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 1u, data, 6u);
        break;

    case 2u:    /* 帧2: 轮速 RR/XL/XR (0.1km/h) */
        data[0] = (uint8_t)(PSWWheelSpeedRR >> 8);
        data[1] = (uint8_t)(PSWWheelSpeedRR & 0xFFu);
        data[2] = (uint8_t)(PSWWheelSpeedXL >> 8);
        data[3] = (uint8_t)(PSWWheelSpeedXL & 0xFFu);
        data[4] = (uint8_t)(PSWWheelSpeedXR >> 8);
        data[5] = (uint8_t)(PSWWheelSpeedXR & 0xFFu);
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 2u, data, 6u);
        break;

    case 3u:    /* 帧3: 轮速传感器故障 (bit0~5: FL/FR/RL/RR/XL/XR) */
        data[0] = (uint8_t)( (PSW_BIT(PSWErrWssOpenFL)  << 0u)
                           | (PSW_BIT(PSWErrWssOpenFR)  << 1u)
                           | (PSW_BIT(PSWErrWssOpenRL)  << 2u)
                           | (PSW_BIT(PSWErrWssOpenRR)  << 3u)
                           | (PSW_BIT(PSWErrWssOpenXL)  << 4u)
                           | (PSW_BIT(PSWErrWssOpenXR)  << 5u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrWssShortFL) << 0u)
                           | (PSW_BIT(PSWErrWssShortFR) << 1u)
                           | (PSW_BIT(PSWErrWssShortRL) << 2u)
                           | (PSW_BIT(PSWErrWssShortRR) << 3u)
                           | (PSW_BIT(PSWErrWssShortXL) << 4u)
                           | (PSW_BIT(PSWErrWssShortXR) << 5u) );
        data[2] = (uint8_t)( (PSW_BIT(PSWErrWssGapFL)   << 0u)
                           | (PSW_BIT(PSWErrWssGapFR)   << 1u)
                           | (PSW_BIT(PSWErrWssGapRL)   << 2u)
                           | (PSW_BIT(PSWErrWssGapRR)   << 3u)
                           | (PSW_BIT(PSWErrWssGapXL)   << 4u)
                           | (PSW_BIT(PSWErrWssGapXR)   << 5u) );
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 3u, data, 3u);
        break;

    case 4u:    /* 帧4: ABS 阀故障 FL/FR/RL/RR (bit0~3: InOpen/InShort/OutOpen/OutShort) */
        data[0] = (uint8_t)( (PSW_BIT(PSWErrInValOpenFL)   << 0u)
                           | (PSW_BIT(PSWErrInValShortFL)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenFL)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortFL) << 3u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrInValOpenFR)   << 0u)
                           | (PSW_BIT(PSWErrInValShortFR)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenFR)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortFR) << 3u) );
        data[2] = (uint8_t)( (PSW_BIT(PSWErrInValOpenRL)   << 0u)
                           | (PSW_BIT(PSWErrInValShortRL)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenRL)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortRL) << 3u) );
        data[3] = (uint8_t)( (PSW_BIT(PSWErrInValOpenRR)   << 0u)
                           | (PSW_BIT(PSWErrInValShortRR)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenRR)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortRR) << 3u) );
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 4u, data, 4u);
        break;

    case 5u:    /* 帧5: ABS 阀 XL/XR + 21口 + 22口 + TR_ASR(仅 PSW 内部) */
        data[0] = (uint8_t)( (PSW_BIT(PSWErrInValOpenXL)   << 0u)
                           | (PSW_BIT(PSWErrInValShortXL)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenXL)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortXL) << 3u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrInValOpenXR)   << 0u)
                           | (PSW_BIT(PSWErrInValShortXR)  << 1u)
                           | (PSW_BIT(PSWErrOutValOpenXR)  << 2u)
                           | (PSW_BIT(PSWErrOutValShortXR) << 3u) );
        data[2] = (uint8_t)( (PSW_BIT(PSWErr21InValveOpen)   << 0u)
                           | (PSW_BIT(PSWErr21InValveShort)  << 1u)
                           | (PSW_BIT(PSWErr21OutValveOpen)  << 2u)
                           | (PSW_BIT(PSWErr21OutValveShort) << 3u) );
        data[3] = (uint8_t)( (PSW_BIT(PSWErr22InValveOpen)   << 0u)
                           | (PSW_BIT(PSWErr22InValveShort)  << 1u)
                           | (PSW_BIT(PSWErr22OutValveOpen)  << 2u)
                           | (PSW_BIT(PSWErr22OutValveShort) << 3u)
                           | (PSW_BIT(PSWErrASRValveOpenX)   << 4u)
                           | (PSW_BIT(PSWErrASRValveShortX)  << 5u) );
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 5u, data, 4u);
        break;

    case 6u:    /* 帧6: 驱动芯片故障 (byte0=Open, byte1=Short, bit0~4=U6/U9/U12/U13/U19) */
        data[0] = (uint8_t)( (PSW_BIT(PSWErrOpenDrive724_U6)  << 0u)
                           | (PSW_BIT(PSWErrOpenDrive724_U9)  << 1u)
                           | (PSW_BIT(PSWErrOpenDrive724_U12) << 2u)
                           | (PSW_BIT(PSWErrOpenDrive724_U13) << 3u)
                           | (PSW_BIT(PSWErrOpenDrive724_U19) << 4u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrShortDrive724_U6)  << 0u)
                           | (PSW_BIT(PSWErrShortDrive724_U9)  << 1u)
                           | (PSW_BIT(PSWErrShortDrive724_U12) << 2u)
                           | (PSW_BIT(PSWErrShortDrive724_U13) << 3u)
                           | (PSW_BIT(PSWErrShortDrive724_U19) << 4u) );
        data[2] = PSWErrDriveChip;   /* 芯片总故障 */
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 6u, data, 3u);
        break;

    case 7u:    /* 帧7: 压力/电源/Relay/EEprom 故障 + 重启原因
                 * (原 bit2/bit3 的 PSWfErrPreF/PreR 已废弃: 该变量无定义也从未被写入,
                 *  压力故障统一由 PSWfErrPreX2_7/X3_10 上报 → bit0/bit1) */
        data[0] = (uint8_t)( (PSW_BIT(PSWfErrPreX2_7)  << 0u)
                           | (PSW_BIT(PSWfErrPreX3_10) << 1u) );
        data[1] = (uint8_t)( (PSW_BIT(PSWErrVpwrLo)        << 0u)
                           | (PSW_BIT(PSWErrVpwrHi)        << 1u)
                           | (PSW_BIT(PSWErrRelay)         << 2u)
                           | (PSW_BIT(PSWfErrEEprom)       << 3u)
                           | (PSW_BIT(PSWfErrEEpromOutBnd) << 4u) );
        data[2] = (uint8_t)(resetReason >> 24);
        data[3] = (uint8_t)(resetReason >> 16);
        data[4] = (uint8_t)(resetReason >> 8);
        data[5] = (uint8_t)(resetReason & 0xFFu);
        (void)can1_sendMsg(PSW_DBG_CAN_ID + 7u, data, 6u);
        break;

    /* ====================================================================== */
    /* ====================================================================== */
    /*  【仅保留 S2】->ValvePwm 写入接口实参 (10us 步, 四阀 U16 大端)             */
    /*    帧10 = period  帧11 = htime  帧17 = 实算: 占空比% + 周期(x2ms)         */
    /*    帧8/9(S1)、帧12~16(S3)、帧18(计数) 已裁剪: 保留帧号但不发送             */
    /* ====================================================================== */
    case 8u:    /* 已裁剪: S1 采样点 */
    case 9u:
    case 12u:   /* 已裁剪: S3 采样点 */
    case 13u:
    case 14u:
    case 15u:
    case 16u:   /* 已裁剪: S1 实算帧 */
    case 18u:   /* 已裁剪: 调用计数帧 */
        break;

    case 10u:   /* 帧10: S2 period (10us 步), 四阀 U16 大端 */
    case 11u:   /* 帧11: S2 htime  (10us 步), 四阀 U16 大端 */
        {
            for (uint8_t i = 0u; i < 4u; i++)
            {
                const uint16_t val = (frame == 10u) ? g_vpwmdbg_period[i] : g_vpwmdbg_htime[i];
                data[i * 2u]      = (uint8_t)(val >> 8);
                data[i * 2u + 1u] = (uint8_t)(val & 0xFFu);
            }
            (void)can1_sendMsg(PSW_DBG_CAN_ID + frame, data, 8u);
        }
        break;

    case 17u:   /* 帧17: S2 实算: 占空比% + 周期(x2ms), 与帧15旧格式一致 */
        {
            for (uint8_t i = 0u; i < 4u; i++)
            {
                const uint16_t per = g_vpwmdbg_period[i];            /* 10us 步 */
                uint32_t p2 = ((uint32_t)per + 100u) / 200u;         /* 10us -> x2ms */
                if (p2 > 255u) { p2 = 255u; }
                data[i]      = dbg_duty_pct(per, g_vpwmdbg_htime[i]);
                data[i + 4u] = (uint8_t)p2;
            }
            (void)can1_sendMsg(PSW_DBG_CAN_ID + 17u, data, 8u);
        }
        break;

    default:
        frame = 0u;
        break;
    }

    frame++;
    if (frame >= PSW_DBG_FRAME_CNT)
    {
        frame = 0u;
    }
}

#endif /* TEST_PSW_CAN_PRINT */

/* ========================================================================== */
/*  压力/VPOWER 10ms CAN 上报 (TEST_PRESSURE_CAN)                              */
/*  CAN1, ID 0x730, 标准帧, DLC 8                                             */
/*                                                                            */
/*  B0-1 前桥/21口压力 PSWBrkPreX2_7 (kPa)   B2-3 后桥/22口压力 PSWBrkPreX3_10 */
/*  B4-5 VPOWER PSWvIgn (0.1V)               B6 bit0/bit1 = 前后桥压力故障     */
/*  B7   保留 0xFF                                                             */
/*  多字节一律大端; 发 PSW* 原始值 (不做 vpwr 门控/掩蔽), 便于标定观察           */
/* ========================================================================== */
#if (TEST_PRESSURE_CAN != 0u)

#define PSWP_CAN_ID     0x730u

static void psw_pressure_can_print(void)
{
    uint8_t data[8];

    data[0] = (uint8_t)(PSWBrkPreX2_7  >> 8);
    data[1] = (uint8_t)(PSWBrkPreX2_7  & 0xFFu);
    data[2] = (uint8_t)(PSWBrkPreX3_10 >> 8);
    data[3] = (uint8_t)(PSWBrkPreX3_10 & 0xFFu);
    data[4] = (uint8_t)(PSWvIgn       >> 8);
    data[5] = (uint8_t)(PSWvIgn       & 0xFFu);
    data[6] = (uint8_t)(((PSWfErrPreX2_7 != 0u) ? 1u : 0u) << 0u)
            | (uint8_t)(((PSWfErrPreX3_10 != 0u) ? 1u : 0u) << 1u);
    data[7] = 0xFFu;

    (void)can1_sendMsg(PSWP_CAN_ID, data, 8u);
}

#endif /* TEST_PRESSURE_CAN */

/* ========================================================================== */
/*  测试层入口                                                                 */
/* ========================================================================== */

/** ③.5~④ 阀类测试 (须晚于 PSWData_Refresh, 早于 ValveDiag_Process) */
void TestDeal_Early(void)
{
#if (TEST_VALVE_HOLD != 0u)
    /* 阀保压阶梯标定: 模块内部每 10ms 调 Bts724g_NotifyValveActive 让电气诊断让路 */
    ValveHoldTest_Process();
#endif

#if (TEST_VALVE_DRV_WAVE != 0u)
    valve_drv_test_single();
#endif
}

/** ⑥ 阀诊断明细输出 (开关在 bts724g.h 的 VALVE_DIAG_DEBUG) */
void TestDeal_Mid(void)
{
#if (VALVE_DIAG_DEBUG != 0u)
    ValveDiag_DebugDump();
#endif
}

#include "PSWdebug.h"   /* 测试用例框架入口 (声明见该头文件) */

/** ⑧.5~⑧.7 CAN 调试上报 + ⑪ 节拍验证 + PSWdebug 用例 (须晚于 PSWDataToRte) */
void TestDeal_Late(void)
{
#if (TEST_RTE_CAN_PRINT != 0u)
    rte_can_print_monitor();
#endif

#if (TEST_PSW_CAN_PRINT != 0u)
    psw_can_print_monitor();
#endif

#if (TEST_PRESSURE_CAN != 0u)
    psw_pressure_can_print();
#endif

#if (TEST_TICK_DEBUG != 0u)
    {
        static uint8_t s_tick_toggle = 0u;
        s_tick_toggle ^= 1u;        /* 每 10ms 翻转一次 */
        TestMark_Ctrl(s_tick_toggle);
    }
#endif

    /* PSWdebug 测试用例: CAN 下发编号 (RTEfDbgMsgSW3) → 注册 → 每 10ms 周期执行
     * 位置与老工程 10ms 主处理中 PSWDataToRte() 之后一致 */
    PSWdebug();
}

#else   /* TEST_DEAL_ENABLE == 0: 空入口, 保证 cmn.c 的调用照常链接 */

void TestDeal_Early(void) { }
void TestDeal_Mid(void)   { }
void TestDeal_Late(void)  { }

#endif /* TEST_DEAL_ENABLE */
