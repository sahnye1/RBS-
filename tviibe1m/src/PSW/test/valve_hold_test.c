/**
 * @file    valve_hold_test.c
 * @brief   21口 / 22口 进气保压测试 — CAN 命令直接驱动 (无状态机)
 *
 * @details 设计要点:
 *   1) 不做 10ms 状态机: 启动命令一次性把 PWM 参数写进 TCPWM, 硬件持续输出波形。
 *   2) 用 RTE 阀函数 (InValActX3_7 / OutValActX3_10 等) 而非直接写寄存器:
 *      RTE 内部已含诊断让路 (Bts724g_NotifyValveActive) + 参数钳位 + 相位连续保持。
 *   3) 排气阀保压期用 100% 恒高 (htime = period): NO 常开阀必须持续通电才关得住,
 *      降占空比 = 降吸力, 带压时只会更关不住。
 *   4) 停止 = 进气 htime=0 (关阀) + 排气 htime=0 (断电, NO 阀自然排空) → 安全排空。
 *
 * @note 低边开关 (P2.0 / P6.2) 由 ASW 侧负责, 本模块不碰。
 */

#include "test_cfg.h"              /* TEST_VALVE_HOLD 编译开关 (0 = 本文件整体不编译) */

#if (TEST_VALVE_HOLD != 0u)

#include "valve_hold_test.h"
#include "RTE.h"                    /* InValActX3_7 / OutValActX3_10 等阀动作接口 */

/* ========================================================================== */
/*  阀映射 (RTE 阀函数: 内部含诊断让路 + 参数钳位 + 恒高/恒低自动处理)        */
/* ========================================================================== */
#if (VHT_CHANNEL == 0u)
    #define VHT_IN_ACT     InValActX3_7      /* 21口进气 (后段高: 每周期先低后高) */
    #define VHT_OUT_ACT    OutValActX3_10    /* 21口排气 (前段高: 每周期先高后低) */
#elif (VHT_CHANNEL == 1u)
    #define VHT_IN_ACT     InValActX4_16     /* 22口进气 (后段高) */
    #define VHT_OUT_ACT    OutValActX2_16    /* 22口排气 (前段高) */
#else
    #error "VHT: invalid VHT_CHANNEL (0=21口, 1=22口)"
#endif

/** ms -> RTE 接口单位 (0.1ms) */
#define VHT_MS_TO_RTE(ms)   ((uint16_t)((ms) * 10u))

/* 上次启动的周期 (0.1ms): 停止时复用 (htime=0 时 period 不影响关阀行为) */
static uint16_t s_vht_period = VHT_MS_TO_RTE(VHT_CYCLE_MAX_MS);

bool ValveHoldTest_CanCmd(uint8_t cmd, uint16_t cycle_ms, uint8_t duty_pct)
{
    if (cmd == 0u)
    {
        /* 停止: 进气 htime=0 -> 关阀; 排气 htime=0 -> 断电 (NO 阀自然排空) */
        VHT_IN_ACT(s_vht_period, 0u);
        VHT_OUT_ACT(s_vht_period, 0u);
        return true;
    }

    /* ---- cmd == 1 (启动): 参数校验 ---- */
    if ((cycle_ms % 10u) != 0u)                                       { return false; }
    if ((cycle_ms < VHT_CYCLE_MIN_MS) || (cycle_ms > VHT_CYCLE_MAX_MS)) { return false; }
    if ((duty_pct < VHT_DUTY_MIN_PCT) || (duty_pct > VHT_DUTY_MAX_PCT))   { return false; }

    {
        const uint16_t period = VHT_MS_TO_RTE(cycle_ms);                    /* 0.1ms */
        uint16_t       htime  = (uint16_t)(((uint32_t)period * duty_pct) / 100u);
        if (htime == 0u) { htime = 1u; }

        s_vht_period = period;

        /* 排气阀 100% 恒高 (htime = period): 持续通电吸住 NO 阀 -> 保住压力 */
        VHT_OUT_ACT(period, period);
        /* 进气阀 PWM (htime = 周期 x 占空比): 每周期进气 -> 阶梯增压 */
        VHT_IN_ACT(period, htime);
    }
    return true;
}

#endif /* TEST_VALVE_HOLD */
