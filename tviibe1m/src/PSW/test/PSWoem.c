/**
 * @file    PSWoem.c
 * @brief   OEM 模式阀控制 — 模式判定 + 阀类型/周期/占空比驱动
 *
 * @details 执行流程 (PSWoem 每 10ms 调用, 位置 PSW_10ms_deal ⑧.6):
 *          ① 读模式: RTEfTestMode == PSWOEM_MODE_OEM(2) 才继续
 *          ② 模式退出: 上一周期是 OEM、本周期不是 → 关闭本模块打开的所有阀
 *          ③ 阀控制: 读 RTEValveType / RTEValvePeriod / RTEValveDuty
 *             - 阀类型 = 0xFF      → 无动作
 *             - 周期或占空比 = 0   → 关闭该阀
 *             - 其他               → 打开该阀, PWM 阀再按占空比设参数
 *
 * @note    · 阀动作统一走 Bts724g_SetValve + Bts724g_NotifyValveActive(active=1),
 *            通知后阀诊断会跳过该阀, 避免 OEM 打阀与阀诊断互相干扰。
 *          · 本模块打开过的阀记在 s_oem_open_mask, 退出 OEM 模式时统一关闭恢复默认。
 *          · PWM 相位: 进气阀 (偶数 ID) 后段高, 排气阀 (奇数 ID) 前段高;
 *            周期单位 0.1ms, 内部换算为 TCPWM 步数 (1 步 = 10us)。
 *          · RTE 变量 (RTEValveType 等) 当前临时定义在本文件, 待 RTE.h 正式加入后迁移。
 *          · 模式变量用 RTEfTestMode, 集中在 PSWmode_GetMode() 一处 (工厂模式共用)。
 */

#include "cmn.h"
#include "bts724g.h"
#include "valve_pwm.h"
#include "RTE.h"
#include "PSWoem.h"

/* ========================================================================== */
/*  0. RTE 阀控制输入 (临时定义 — 待 RTE.h 正式加入同名 extern 后移至 RTE.c)     */
/*                                                                            */
/*  COM/ASW 层写入 → PSW 层执行 PWM; 当前由 OEM 模式使用。                       */
/* ========================================================================== */
uint8_t  RTEValveType   = PSWOEM_VALVE_NONE;  /* 阀类型: valve_id_t 值 (0~15), 0xFF = 无动作 */
uint16_t RTEValvePeriod = 0u;                 /* PWM 周期: 单位 0.1ms                        */
uint8_t  RTEValveDuty   = 0u;                 /* 占空比: 0~100 %, 0=常关 100=常开             */

/* ========================================================================== */
/*  1. 运行时状态                                                               */
/* ========================================================================== */

static uint8_t  s_oem_last_mode = PSWOEM_MODE_NORMAL;  /* 上一周期模式 (判进入/退出)      */
static uint32_t s_oem_open_mask = 0u;                  /* 本模块打开的阀位图 (bit=valve_id) */

/* ========================================================================== */
/*  2. 模式通道 (RTEfTestMode; 工厂模式共用, 声明见 PSWoem.h)                    */
/* ========================================================================== */

/** @brief 读当前模式 (RTEfTestMode: 0 正常 / 1 工厂 / 2 OEM) */
uint8_t PSWmode_GetMode(void)
{
    return RTEfTestMode;
}

/* ========================================================================== */
/*  3. 阀动作                                                                   */
/* ========================================================================== */

/** @brief 关闭全部阀并清空标记 (退出 OEM 模式时调用)
 *  @note  对所有阀调一次 SetValve(false), 已关闭的阀重复关闭无副作用。
 */
static void oem_all_off(void)
{
    uint8_t vid;

    for (vid = 0u; vid < VALVE_NUM_TOTAL; vid++)
    {
        Bts724g_SetValve((valve_id_t)vid, false);
        Bts724g_NotifyValveActive((valve_id_t)vid, false);
    }
    s_oem_open_mask = 0u;
}

/** @brief 打开指定阀 (并通知阀诊断跳过该阀)
 *  @param vid 阀索引 (valve_id_t, 0~VALVE_NUM_TOTAL-1), 越界忽略
 */
static void oem_valve_on(uint8_t vid)
{
    if (vid >= VALVE_NUM_TOTAL)
    {
        return;
    }
    Bts724g_NotifyValveActive((valve_id_t)vid, true);
    Bts724g_SetValve((valve_id_t)vid, true);
    s_oem_open_mask |= (uint32_t)(1ul << vid);
}

/** @brief 关闭指定阀 (并解除阀诊断跳过)
 *  @param vid 阀索引 (valve_id_t, 0~VALVE_NUM_TOTAL-1), 越界忽略
 */
static void oem_valve_off(uint8_t vid)
{
    if (vid >= VALVE_NUM_TOTAL)
    {
        return;
    }
    Bts724g_SetValve((valve_id_t)vid, false);
    Bts724g_NotifyValveActive((valve_id_t)vid, false);
    s_oem_open_mask &= (uint32_t)~(uint32_t)(1ul << vid);
}

/** @brief 按 周期/占空比 驱动指定阀
 *  @param vid        阀索引 (调用方已保证 < VALVE_NUM_TOTAL)
 *  @param period01ms PWM 周期 (0.1ms); 0 = 关闭该阀
 *  @param dutyPct    占空比 (0~100 %); 0 = 关闭该阀
 *  @note  进气阀 (偶数 ID) 后段高, 排气阀 (奇数 ID) 前段高; 非 PWM 阀只做开关。
 */
static void oem_valve_set(uint8_t vid, uint16_t period01ms, uint8_t dutyPct)
{
    uint16_t periodTicks;
    uint16_t htimeTicks;
    uint16_t cc0;

    if ((period01ms == 0u) || (dutyPct == 0u))
    {
        oem_valve_off(vid);
        return;
    }

    oem_valve_on(vid);

    if (!ValvePwm_IsPwmValve((valve_id_t)vid))
    {
        return;    /* 非 PWM 阀: oem_valve_on 已把它打开 */
    }

    periodTicks = (uint16_t)(period01ms * 10u);                        /* 0.1ms → 10us 步 */
    htimeTicks  = (uint16_t)((uint32_t)periodTicks * dutyPct / 100u);
    cc0 = ((vid & 1u) == 0u) ? (uint16_t)(periodTicks - htimeTicks) : 0u;
    ValvePwm_SetDutyLive((valve_id_t)vid, periodTicks, cc0, htimeTicks);
}

/* ========================================================================== */
/*  4. 10ms 入口                                                                */
/* ========================================================================== */

/** @brief OEM 模式阀控制入口 (每 10ms 调用一次)
 *  @note  非 OEM 模式时只做一次退出恢复, 不读取/不消费阀控制变量。
 */
void PSWoem(void)
{
    uint8_t mode = PSWmode_GetMode();
    uint8_t vid;

    /* ① 模式退出: 关闭本模块打过的阀, 恢复默认状态 */
    if ((s_oem_last_mode == PSWOEM_MODE_OEM) && (mode != PSWOEM_MODE_OEM))
    {
        oem_all_off();
    }
    s_oem_last_mode = mode;

    /* ② 非 OEM 模式不接管 */
    if (mode != PSWOEM_MODE_OEM)
    {
        return;
    }

    /* ③ 阀控制: 阀类型 + 周期 + 占空比 */
    vid = RTEValveType;
    if (vid == PSWOEM_VALVE_NONE)
    {
        return;
    }
    RTEValveType = PSWOEM_VALVE_NONE;    /* 消费, 防止同一请求被每周期重复执行 */

    if (vid >= VALVE_NUM_TOTAL)
    {
        return;
    }

    oem_valve_set(vid, RTEValvePeriod, RTEValveDuty);
}
