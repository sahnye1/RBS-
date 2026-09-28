/**
 * @file    valve_hold_test.c
 * @brief   阶梯进气保压标定测试 — 10ms 非阻塞状态机
 *
 * @details 阀映射 (VHT_CHANNEL 选通道, VHT_USE_FRONT_VALVES 选原阀/前桥阀):
 *    21口 (CHANNEL=0): 进气 VALVE_21IN,  排气 VALVE_21OUT  (U12, X3_7/X3_10), 压力 PSWBrkPreX2_7  ← 当前
 *    22口 (CHANNEL=1): 进气 VALVE_22IN,  排气 VALVE_22OUT  (U13, X4_16/X2_16), 压力 PSWBrkPreX3_10
 *    低边: 21口 → P2.0; 22口 → P2.0 + P6.2 (见 vht_lowside_enable)
 *
 * @details 流程:
 *   PREP  → 条件检查 + 开低边 + 排气组通电关闭(NO) + 启动进气组周期脉冲
 *   STEP  → 每 VHT_CYCLE_MS(当前 270ms = 20ms 进气 + 250ms 保压) 一步:
 *           进气脉冲 → 关阀保压 → 采样 P_a(关阀后 20ms)/P_b(步末)
 *   FINAL → 关进气组, 长保压观察 (默认 1s)
 *   VENT  → 全部断电, 排气阀常开排空, 等压力 < 50kPa
 *   DONE  → 恢复诊断让路/低边, 出结果
 *
 * @details 关键实现点:
 *   1) 进气脉宽由 TCPWM 产生 (ValvePwm_SetDuty 归零 → 相位从调用时刻起算),
 *      软件不参与脉宽计时 (主循环 10ms 节拍做不到 ms 级精度)。
 *   2) 排气组为 NO 常开阀, 测试全程必须保持通电 (SetValve true) 才关得住。
 *   3) 每个 10ms 节拍都要覆盖 Bts724g_NotifyValveActive(true), 否则
 *      ValveDiag_Process (40ms 轮转) 会插入检阀脉冲污染阶梯进气。
 *   4) 任何异常/复位都是全部断电 → 进阀关 + 出阀开 → 自动排空 (fail-safe)。
 */

#include "valve_hold_test.h"

#include "bts724g.h"
#include "valve_pwm.h"
#include "gpio.h"
#include "psw_data.h"
#include "can.h"

/* ========================================================================== */
/*  内部宏与全局                                                               */
/* ========================================================================== */

/* PWM 步 = 10μs → 1ms = 100 步 */
#define VHT_MS_TO_STEPS(ms)         ((uint16_t)((ms) * 100u))

/* VPOWER 有效窗口 (0.1V), 与 adc/psw_data 门控一致 */
#define VHT_VPWR_MIN                180u
#define VHT_VPWR_MAX                320u

volatile vht_data_t g_vht = {0};

/* 启动条件等待计数 (上电初期 VPOWER/压力未就绪时逐拍重试, 不立即放弃) */
static uint16_t s_prep_wait_tick = 0u;

/* ========================================================================== */
/*  阀映射 (进气组 / 排气组)                                                   */
/*                                                                            */
/*  线缆已于 2026-09-15 改接回 21口/22口原阀 (U12/U13):                        */
/*    21口 → VALVE_21IN / VALVE_21OUT  (X3_7 / X3_10)                         */
/*    22口 → VALVE_22IN / VALVE_22OUT  (X4_16 / X2_16)                        */
/*  VHT_USE_FRONT_VALVES 置 1 可临时切回前桥阀 (LFI/LFO、RFI/RFO) 代替。        */
/* ========================================================================== */

#if (VHT_CHANNEL == 0u)                 /* ---- 21口 () ---- */
    #if (VHT_USE_FRONT_VALVES != 0u)
        #define VHT_IN_VALVE            VALVE_LFI       /* 左前进气 (代替 VALVE_21IN) */
        #define VHT_OUT_VALVE           VALVE_LFO       /* 左前排气 (代替 VALVE_21OUT) */
    #else
        #define VHT_IN_VALVE            VALVE_21IN
        #define VHT_OUT_VALVE           VALVE_21OUT
    #endif
    #define VHT_PRESSURE_KPA()          PSWBrkPreX2_7
    #define VHT_PRESSURE_ERR()          PSWfErrPreX2_7

#elif (VHT_CHANNEL == 1u)               /* ---- 22口 (当前待测) ---- */
    #if (VHT_USE_FRONT_VALVES != 0u)
        #define VHT_IN_VALVE            VALVE_RFI       /* 右前进气 (代替 VALVE_22IN) */
        #define VHT_OUT_VALVE           VALVE_RFO       /* 右前排气 (代替 VALVE_22OUT) */
    #else
        #define VHT_IN_VALVE            VALVE_22IN
        #define VHT_OUT_VALVE           VALVE_22OUT
    #endif
    #define VHT_PRESSURE_KPA()          PSWBrkPreX3_10  /* 22口压力源待确认 */
    #define VHT_PRESSURE_ERR()          PSWfErrPreX3_10
#else
    #error "VHT: invalid VHT_CHANNEL (0=21口, 1=22口)"
#endif

static const valve_id_t s_in_valves[]  = { VHT_IN_VALVE  };
static const valve_id_t s_out_valves[] = { VHT_OUT_VALVE };

#define VHT_IN_CNT      ((uint8_t)(sizeof(s_in_valves)  / sizeof(s_in_valves[0])))
#define VHT_OUT_CNT     ((uint8_t)(sizeof(s_out_valves) / sizeof(s_out_valves[0])))

/* ========================================================================== */
/*  内部辅助                                                                   */
/* ========================================================================== */

/** 保压容差换算: VHT_P_TOL_KPA_100MS (kPa/100ms) → 实际采样间隔对应的 kPa */
static uint16_t vht_tol_kpa(uint16_t ms)
{
    uint32_t t = ((uint32_t)VHT_P_TOL_KPA_100MS * (uint32_t)ms) / 100u;
    if (t == 0u)
    {
        t = 1u;     /* 至少给 1kPa, 避免间隔过短导致恒失败 */
    }
    return (uint16_t)t;
}

static int16_t vht_abs16(int16_t v)
{
    return (v < 0) ? (int16_t)(-v) : v;
}

/* 阀极性电平 (见 VHT_VALVE_LOGIC 说明) */
#define VHT_IN_OPEN_LEVEL       ((VHT_VALVE_LOGIC != 0u) ? false : true)   /* 进气阀"打开"电平 */
#define VHT_IN_CLOSE_LEVEL      ((VHT_VALVE_LOGIC != 0u) ? true  : false)  /* 进气阀"关闭"电平 */
#define VHT_OUT_CLOSE_LEVEL     ((VHT_VALVE_LOGIC != 0u) ? false : true)   /* 排气阀"关闭(保压)"电平 */
#define VHT_OUT_OPEN_LEVEL      ((VHT_VALVE_LOGIC != 0u) ? true  : false)  /* 排气阀"打开(排空)"电平 */

/** 进气组关闭 */
static void vht_in_close(void)
{
    uint8_t i;
    for (i = 0u; i < VHT_IN_CNT; i++)
    {
        Bts724g_SetValve(s_in_valves[i], VHT_IN_CLOSE_LEVEL);
    }
}

#if (VHT_CONTINUOUS_MODE != 0u)
/** 进气组常开 (仅连续充压模式使用, 故条件编译避免未引用警告) */
static void vht_in_open(void)
{
    uint8_t i;
    for (i = 0u; i < VHT_IN_CNT; i++)
    {
        Bts724g_SetValve(s_in_valves[i], VHT_IN_OPEN_LEVEL);
    }
}
#endif

/** 进气组启动周期脉冲: 每周期"打开" VHT_IN_PULSE_MS, 周期 VHT_CYCLE_MS (相位从本刻起)
 *
 *  两种极性都保证"打开窗口落在周期开头 0~PULSE", 采样相位一致:
 *    logic 0 (进气 NC, 打开=高): cc0=0,              high=PULSE → 0~PULSE 高
 *    logic 1 (进气 NO, 打开=低): cc0=PULSE, high=CYCLE(CC1溢出) → 0~PULSE 低, 之后恒高 */
static void vht_in_pulse_start(void)
{
    uint8_t  i;
    uint16_t cc0;
    uint16_t hi;

    if (VHT_VALVE_LOGIC != 0u)
    {
        cc0 = VHT_MS_TO_STEPS(VHT_IN_PULSE_MS);
        hi  = VHT_MS_TO_STEPS(VHT_CYCLE_MS);        /* CC1 溢出 → 恒高到周期末 */
    }
    else
    {
        cc0 = 0u;
        hi  = VHT_MS_TO_STEPS(VHT_IN_PULSE_MS);
    }

    for (i = 0u; i < VHT_IN_CNT; i++)
    {
        ValvePwm_SetDuty(s_in_valves[i], VHT_MS_TO_STEPS(VHT_CYCLE_MS), cc0, hi);
    }
}

/** 排气组关闭 (保住气压) */
static void vht_out_close(void)
{
    uint8_t i;
    for (i = 0u; i < VHT_OUT_CNT; i++)
    {
        Bts724g_SetValve(s_out_valves[i], VHT_OUT_CLOSE_LEVEL);
    }
}

/** 排气组打开 (排空) */
static void vht_out_open(void)
{
    uint8_t i;
    for (i = 0u; i < VHT_OUT_CNT; i++)
    {
        Bts724g_SetValve(s_out_valves[i], VHT_OUT_OPEN_LEVEL);
    }
}

/** 排气组"保压保持"驱动 (占空比可调, 见 valve_hold_test.h 的 VHT_OUT_HOLD_DUTY_PCT):
 *    PCT = 100 → 直流恒高 (SetOnOff(true): 真恒高、吸力最大)
 *    PCT < 100 → 斩波保持: 周期 VHT_OUT_CHOP_PERIOD_MS, 高电平占 PCT%
 *                (前段高: CC0=0 起点 SET / CC1=htime 终点 CLEAR → 0~htime 为高)
 *  ⚠ 这里用 #if 而不是运行时 if: 另一种配置的代码不参与编译,
 *     否则 PCT=100 时会报 Warning[Pe111] statement is unreachable (2026-09-16) */
static void vht_out_hold(void)
{
#if (VHT_OUT_HOLD_DUTY_PCT >= 100u)
    vht_out_close();                    /* 100%: 直流恒高 */
#else
    uint8_t  i;
    uint16_t period_steps;
    uint16_t htime_steps;

    period_steps = (uint16_t)(VHT_OUT_CHOP_PERIOD_MS * 100u);                   /* 10μs/步 */
    htime_steps  = (uint16_t)(((uint32_t)period_steps * VHT_OUT_HOLD_DUTY_PCT) / 100u);

    if (htime_steps == 0u)            { htime_steps = 1u; }                     /* 0% 会成恒低, 至少 1 步 */
    if (htime_steps >= period_steps)  { htime_steps = (uint16_t)(period_steps - 1u); }

    for (i = 0u; i < VHT_OUT_CNT; i++)
    {
        ValvePwm_SetDuty(s_out_valves[i], period_steps, 0u, htime_steps);
    }
#endif
}

/** 安全态: 进气关闭 + 排气打开 → 排空 */
static void vht_all_safe(void)
{
    vht_in_close();
    vht_out_open();
}

/** 低边开关使能 (按当前测试通道, 引用计数式):
 *    21口 (VHT_CHANNEL=0) → P2.0 (ASRF, 原 FA_ASR 低边)
 *    22口 (VHT_CHANNEL=1) → P2.0 (22口进气阀) + **P6.2** (ASRR, 原 DA_ASR 低边 → 22口排气阀)
 *  ⚠ 排气阀是 NO 常开阀: 若其回地低边未开, 它无法通电关闭 → 完全保不住压 */
static void vht_lowside_enable(uint8_t enable)
{
#if (VHT_LOWSIDE_ENABLE != 0u)
    Gpio_ASRFLowSideEnable(enable);         /* P2.0 */
#if (VHT_CHANNEL == 1u)
    Gpio_ASRRLowSideEnable(enable);         /* P6.2 (仅 22口) */
#endif
#else
    (void)enable;
#endif
}

/** 保持电气诊断让路 (覆盖 Psw_*ValAct 内部可能的 NotifyValveActive(false)) */
static void vht_keep_diag_off(void)
{
    uint8_t i;
    for (i = 0u; i < VHT_IN_CNT;  i++) { Bts724g_NotifyValveActive(s_in_valves[i],  true); }
    for (i = 0u; i < VHT_OUT_CNT; i++) { Bts724g_NotifyValveActive(s_out_valves[i], true); }
}

/** 恢复电气诊断 */
static void vht_diag_restore(void)
{
    uint8_t i;
    for (i = 0u; i < VHT_IN_CNT;  i++) { Bts724g_NotifyValveActive(s_in_valves[i],  false); }
    for (i = 0u; i < VHT_OUT_CNT; i++) { Bts724g_NotifyValveActive(s_out_valves[i], false); }
}

/* ========================================================================== */
/*  CAN 实时上报 (CAN1 调试口, 上位机画曲线用)                                  */
/*                                                                            */
/*  0x720 实时曲线帧 (每 10ms):                                                */
/*    B0 state | B1 step | B2-3 压力kPa(大端) | B4 tick(步内拍)                 */
/*    B5 bit0=进气阀电平 bit1=排气阀电平 (1=高电平)                              */
/*    B6-7 时间 tick (大端, ×10 = ms, 从测试启动起)                             */
/*                                                                            */
/*  0x721 每步结果帧 (每步末):                                                  */
/*    B0 step | B1-2 p_step(kPa) | B3-4 d_hold(int16 kPa) | B5 bit0=漏 bit1=满  */
/*    B6-7 dp_step (int16 kPa, 本步实际升压 ΔP, 大端) ← 标定脉宽↔升压看这个      */
/*                                                                            */
/*  0x722 最终汇总帧 (结束):                                                    */
/*    B0 result | B1 steps_done | B2-3 p_final_start | B4-5 p_final_end         */
/*    B6 bit0=充满 bit1=保压超差                                                */
/* ========================================================================== */
#if (VHT_CAN_REPORT_ENABLE != 0u)

static void vht_can_trace(void)
{
    uint8_t  data[8];
    uint16_t p    = VHT_PRESSURE_KPA();
    uint8_t  in_lvl;
    uint8_t  out_lvl;
    uint8_t  pulse_on = ((g_vht.state == VHT_STEP) &&
                         ((g_vht.tick * 10u) < VHT_IN_PULSE_MS)) ? 1u : 0u;

#if (VHT_CONTINUOUS_MODE != 0u)
    if (g_vht.state == VHT_CHARGE)
    {
        pulse_on = 1u;      /* 连续模式: 充气阶段阀常开 */
    }
#endif

#if (VHT_VALVE_LOGIC == 0u)
    in_lvl  = pulse_on;                             /* 进气 NC: 脉冲期=高=开 */
    out_lvl = (g_vht.state == VHT_VENT) ? 0u : 1u;  /* 排气 NO: 非排空期=高=关 */
#else
    in_lvl  = (uint8_t)(pulse_on ^ 1u);
    out_lvl = (g_vht.state == VHT_VENT) ? 1u : 0u;
#endif

    data[0] = g_vht.state;
    data[1] = g_vht.step;
    data[2] = (uint8_t)(p >> 8);
    data[3] = (uint8_t)(p & 0xFFu);
    data[4] = (uint8_t)g_vht.tick;
    data[5] = (uint8_t)(((in_lvl & 1u) << 0u) | ((out_lvl & 1u) << 1u));
    data[6] = (uint8_t)((g_vht.total_tick >> 8) & 0xFFu);
    data[7] = (uint8_t)(g_vht.total_tick & 0xFFu);

    (void)can1_sendMsg(VHT_CAN_ID_TRACE, data, 8u);
}

static void vht_can_step(uint8_t idx)
{
    uint8_t data[8];
    uint16_t d  = (uint16_t)g_vht.d_hold[idx];
    uint16_t dp = (uint16_t)g_vht.dp_step[idx];

    data[0] = (uint8_t)(idx + 1u);
    data[1] = (uint8_t)(g_vht.p_step[idx] >> 8);
    data[2] = (uint8_t)(g_vht.p_step[idx] & 0xFFu);
    data[3] = (uint8_t)(d >> 8);
    data[4] = (uint8_t)(d & 0xFFu);
    data[5] = (uint8_t)(((g_vht.leak_flag ? 1u : 0u) << 0u) |
                        ((g_vht.full_flag ? 1u : 0u) << 1u));
    data[6] = (uint8_t)(dp >> 8);            /* 本步升压 ΔP (int16, 大端) */
    data[7] = (uint8_t)(dp & 0xFFu);

    (void)can1_sendMsg(VHT_CAN_ID_STEP, data, 8u);
}

static void vht_can_result(void)
{
    uint8_t data[8];

    data[0] = g_vht.result;
    data[1] = g_vht.steps_done;
    data[2] = (uint8_t)(g_vht.p_final_start >> 8);
    data[3] = (uint8_t)(g_vht.p_final_start & 0xFFu);
    data[4] = (uint8_t)(g_vht.p_final_end >> 8);
    data[5] = (uint8_t)(g_vht.p_final_end & 0xFFu);
    data[6] = (uint8_t)(((g_vht.full_flag ? 1u : 0u) << 0u) |
                        ((g_vht.leak_flag ? 1u : 0u) << 1u));
    data[7] = 0xFFu;

    (void)can1_sendMsg(VHT_CAN_ID_RESULT, data, 8u);
}
#endif /* VHT_CAN_REPORT_ENABLE */

/** 收尾: 断电 + 恢复低边/诊断让路 + 出结果 */
static void vht_finish(uint8_t result)
{
    vht_all_safe();

    vht_lowside_enable(0u);         /* 释放低边 (P2.0, 22口 再加 P6.2) */

    vht_diag_restore();

    g_vht.result = result;
    g_vht.state  = VHT_DONE;

#if (VHT_CAN_REPORT_ENABLE != 0u)
    vht_can_result();
#endif
}

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

void ValveHoldTest_Start(void)
{
    uint8_t i;

    if ((g_vht.state != VHT_IDLE) && (g_vht.state != VHT_DONE))
    {
        return;     /* 运行中则忽略 (结束后可再次 Start 重跑) */
    }

    g_vht.state      = VHT_IDLE;
    g_vht.step       = 0u;
    g_vht.steps_done = 0u;
    g_vht.tick       = 0u;
    g_vht.total_tick = 0u;
    s_prep_wait_tick = 0u;      /* 重新开始等待启动条件 */

    g_vht.p_init        = 0u;
    g_vht.p_a           = 0u;
    g_vht.p_b           = 0u;
    g_vht.p_final_start = 0u;
    g_vht.p_final_end   = 0u;
    g_vht.p_final       = 0u;

    g_vht.full_flag = 0u;
    g_vht.leak_flag = 0u;
    g_vht.result    = VHT_RES_NONE;

    for (i = 0u; i < VHT_STEP_NUM; i++)
    {
        g_vht.p_step[i]  = 0u;
        g_vht.d_hold[i]  = 0;
        g_vht.dp_step[i] = 0;
    }

    g_vht.step  = 1u;
    g_vht.state = VHT_PREP;
}

void ValveHoldTest_Abort(void)
{
    if ((g_vht.state == VHT_IDLE) || (g_vht.state == VHT_DONE))
    {
        return;
    }
    vht_finish(VHT_RES_ABORT);
}

bool ValveHoldTest_IsRunning(void)
{
    return ((g_vht.state != VHT_IDLE) && (g_vht.state != VHT_DONE));
}

/* ========================================================================== */
/*  10ms 状态机                                                                */
/* ========================================================================== */

void ValveHoldTest_Process(void)
{
#if (VHT_AUTO_START != 0u)
    static bool s_auto_started = false;
    if (!s_auto_started)
    {
        s_auto_started = true;
        ValveHoldTest_Start();
    }
#endif

    if ((g_vht.state == VHT_IDLE) || (g_vht.state == VHT_DONE))
    {
        return;
    }

    /* 总超时保护 */
    g_vht.total_tick++;
    if ((g_vht.total_tick * 10u) > VHT_TOTAL_TIMEOUT_MS)
    {
        vht_finish(VHT_RES_ABORT);
        return;
    }

    /* 诊断让路 (每拍覆盖) */
    vht_keep_diag_off();

    switch (g_vht.state)
    {
    /* ---------------- 准备 ---------------- */
    case VHT_PREP:
    {
        uint8_t i;
        bool    cond_ok = true;

        /* 启动条件 */
        if ((PSWvIgn < VHT_VPWR_MIN) || (PSWvIgn > VHT_VPWR_MAX))
        {
            cond_ok = false;
        }
        if (VHT_PRESSURE_ERR() != 0u)
        {
            cond_ok = false;
        }
        for (i = 0u; i < VHT_IN_CNT; i++)
        {
            if (g_bts724g_fault_status.bts724g_fault_type[s_in_valves[i]] != VALVE_FAULT_NONE)
            {
                cond_ok = false;
            }
        }
        for (i = 0u; i < VHT_OUT_CNT; i++)
        {
            if (g_bts724g_fault_status.bts724g_fault_type[s_out_valves[i]] != VALVE_FAULT_NONE)
            {
                cond_ok = false;
            }
        }
        if (!cond_ok)
        {
            /* 上电初期 VPOWER (PSWvIgn) / 压力信号尚未就绪 → 逐拍等待重试, 不立即放弃。
             * (2026-09-10 修: 原来第一拍就 COND_FAIL 结束, 现象是"打开开关但没运行") */
            if (s_prep_wait_tick < 0xFFFFu)
            {
                s_prep_wait_tick++;
            }
            if ((s_prep_wait_tick * 10u) > VHT_PREP_WAIT_MS)
            {
                vht_finish(VHT_RES_COND_FAIL);
            }
            break;
        }
        s_prep_wait_tick = 0u;

        /* 开低边: 21口 → P2.0; 22口 → P2.0 + P6.2 (排气阀必须能通电关闭) */
        vht_lowside_enable(1u);

        /* 排气组关闭 (保压), 全程保持直到排空阶段
         * (占空比由 VHT_OUT_HOLD_DUTY_PCT 决定: 100% = 直流恒高; <100% = 斩波保持) */
        vht_out_hold();

#if (VHT_CONTINUOUS_MODE != 0u)
        /* 连续全开: 进气阀常开不关 (无周期脉冲, 无保压采样窗口) */
        vht_in_open();

        g_vht.p_init = VHT_PRESSURE_KPA();
        g_vht.step   = 1u;
        g_vht.tick   = 0u;
        g_vht.state  = VHT_CHARGE;
#else
        /* 阶梯脉冲: 每周期开头 VHT_IN_PULSE_MS 打开、其余关闭
         * (波形见 vht_in_pulse_start, 两种极性自动取反)。
         * 用 SetDuty(归零) 而非 SetDutyNoReset: 保证周期起点 = 本刻, 采样相位才确定。 */
        vht_in_pulse_start();

        g_vht.p_init = VHT_PRESSURE_KPA();
        g_vht.step   = 1u;
        g_vht.tick   = 1u;      /* tick=n ↔ PWM 周期起点后 n*10ms (本拍已在起点后 10ms) */
        g_vht.state  = VHT_STEP;
#endif
        break;
    }

    /* ---------------- 阶梯进气 + 保压采样 ---------------- */
    case VHT_STEP:
    {
        if ((g_vht.step < 1u) || (g_vht.step > VHT_STEP_NUM))
        {
            vht_finish(VHT_RES_ABORT);      /* 步号异常, 不应发生 */
            break;
        }

        const uint8_t idx = (uint8_t)(g_vht.step - 1u);

        /* 保压起点采样 (关阀后 VHT_SAMPLE_A_DELAY_MS) */
        if (g_vht.tick == VHT_TICK_SAMPLE_A)
        {
            g_vht.p_a = VHT_PRESSURE_KPA();
        }

        /* 保压终点采样 (本步末, 下次脉冲前 10ms) */
        if (g_vht.tick == VHT_TICK_SAMPLE_B)
        {
            int16_t  d;
            int16_t  p_prev;
            uint16_t tol;

            g_vht.p_b = VHT_PRESSURE_KPA();
            g_vht.p_step[idx] = g_vht.p_b;

            /* 本步实际升压 ΔP (第 1 步基准 = 起始压力 p_init)
             * ← 标定"脉宽 ↔ 每步升多少 kPa"直接看这个 (目标 5kPa / 10kPa 一阶)
             * 拆成两步读, 避免同一表达式里连续访问 volatile (IAR Pa082) */
            p_prev = (int16_t)((idx == 0u) ? g_vht.p_init : g_vht.p_step[idx - 1u]);
            g_vht.dp_step[idx] = (int16_t)g_vht.p_b - p_prev;

            d = (int16_t)g_vht.p_b - (int16_t)g_vht.p_a;
            g_vht.d_hold[idx] = d;

            tol = vht_tol_kpa(VHT_HOLD_SAMPLE_MS);
            if (vht_abs16(d) > (int16_t)tol)
            {
                g_vht.leak_flag = 1u;
            }

            if (g_vht.p_b >= VHT_P_FULL_KPA)
            {
                g_vht.full_flag = 1u;
            }
        }

        g_vht.tick++;
        if (g_vht.tick >= VHT_TICK_PER_STEP)
        {
            /* 归 1 (不是 0): 保持 tick=n ↔ 周期起点后 n*10ms 的相位关系 */
            g_vht.tick       = 1u;
            g_vht.steps_done = g_vht.step;

#if (VHT_CAN_REPORT_ENABLE != 0u)
            vht_can_step(idx);
#endif

            /* 已充满 或 次数用尽 → 转入末次长保压 */
            if ((g_vht.full_flag != 0u) || (g_vht.step >= VHT_STEP_NUM))
            {
                vht_in_close();                             /* 停止进气 */
                g_vht.p_final_start = VHT_PRESSURE_KPA();
                g_vht.p_final       = g_vht.p_final_start;
                g_vht.tick          = 0u;
                g_vht.state         = VHT_FINAL;
            }
            else
            {
                g_vht.step++;
            }
        }
        break;
    }

    /* ---------------- 连续全开充气 (VHT_CONTINUOUS_MODE=1) ---------------- */
    case VHT_CHARGE:
    {
        g_vht.tick++;
        g_vht.p_final = VHT_PRESSURE_KPA();

        if (g_vht.p_final >= VHT_P_FULL_KPA)
        {
            g_vht.full_flag = 1u;
        }

        /* 达到充满阈值 或 到达时长上限 → 关进气阀, 转入长保压测泄漏 */
        if ((g_vht.full_flag != 0u) || ((g_vht.tick * 10u) >= VHT_CONT_MAX_MS))
        {
            vht_in_close();
            g_vht.steps_done    = 1u;
            g_vht.p_final_start = g_vht.p_final;
            g_vht.tick          = 0u;
            g_vht.state         = VHT_FINAL;
        }
        break;
    }

    /* ---------------- 末次长保压观察 ---------------- */
    case VHT_FINAL:
    {
        g_vht.tick++;
        g_vht.p_final = VHT_PRESSURE_KPA();

        if (g_vht.tick >= (VHT_FINAL_HOLD_MS / 10u))
        {
            /* 容差按 20kPa/100ms 折算: 1s 观察窗 → 200kPa (偏松, 标定后可收紧
             * VHT_P_TOL_KPA_100MS 或缩短 VHT_FINAL_HOLD_MS) */
            int16_t d = (int16_t)g_vht.p_final - (int16_t)g_vht.p_final_start;
            if (vht_abs16(d) > (int16_t)vht_tol_kpa(VHT_FINAL_HOLD_MS))
            {
                g_vht.leak_flag = 1u;
            }

            /* 保存长保压终点 (进入排空前, 否则会被排空过程覆盖) */
            g_vht.p_final_end = g_vht.p_final;

            /* 排空: 安全态 = 进气关闭 + 排气打开 (logic0 时即两阀断电) */
            vht_all_safe();
            g_vht.tick  = 0u;
            g_vht.state = VHT_VENT;
        }
        break;
    }

    /* ---------------- 排空 ---------------- */
    case VHT_VENT:
    {
        g_vht.tick++;
        g_vht.p_final = VHT_PRESSURE_KPA();

        if (VHT_PRESSURE_KPA() <= VHT_P_VENT_DONE_KPA)
        {
            int16_t rise = (int16_t)g_vht.p_final_start - (int16_t)g_vht.p_init;

            if (rise < (int16_t)VHT_P_MIN_RISE_KPA)
            {
                vht_finish(VHT_RES_NO_RISE);    /* 压力几乎没升: 阀未动作/气路不通/低边未开 */
            }
            else if (g_vht.leak_flag != 0u)
            {
                vht_finish(VHT_RES_LEAK);
            }
            else if (g_vht.full_flag == 0u)
            {
                vht_finish(VHT_RES_NOT_FULL);
            }
            else
            {
                vht_finish(VHT_RES_OK);
            }
            break;
        }

        if ((g_vht.tick * 10u) >= VHT_VENT_TIMEOUT_MS)
        {
            vht_finish(VHT_RES_VENT_FAIL);
        }
        break;
    }

    default:
        vht_finish(VHT_RES_ABORT);
        break;
    }

#if (VHT_CAN_REPORT_ENABLE != 0u)
    /* 每拍上报一次实时曲线数据 */
    vht_can_trace();
#endif
}
