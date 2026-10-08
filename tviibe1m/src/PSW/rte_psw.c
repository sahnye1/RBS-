/**
 * @file    rte_psw.c
 * @brief   RTE PSW 层实现 — 驾驶室版 (阀控 / 低边 / CAN / EEPROM / 桩函数)
 *
 * @details 阀控: 17 阀 → BTS724G + ValvePwm; RTE 层单位 0.1ms, ValvePwm 步长 10μs
 *          (1 个 RTE 单位 = 10 步; RTE 周期上限 5000 = 500ms)
 *          双比较: CC0→SET(起点) CC1→CLEAR(终点) overflow→CLEAR(基态)
 *            进气阀 (后段高): CC0 = period - htime
 *            排气阀 (前段高): CC0 = 0 (周期起点)
 *          对外接口名 (RTE 2026-09-11 按接插件引脚): 21口 → X3_7/X3_10,
 *            22口 → X4_16/X2_16; 旧名 InValAct21/22 保留兼容 (见文件末尾)
 */

#include "RTE.h"
#include "bts724g.h"
#include "valve_pwm.h"
#include "eeprom.h"
#include "gpio.h"
#include "psw_data.h"
#include <string.h>

/* ========================================================================== */
/*  常量与宏                                                                   */
/* ========================================================================== */

/* RTE 0.1ms → ValvePwm 10μs 步数 */
#define RTE_TO_PWM_STEPS(v)      ((uint16_t)((v) * 10u))

/* RTE 层周期上限 (单位 0.1ms): 5000 = 500ms */
#define RTE_PERIOD_MAX           5000u

/** @brief RTE 阀控参数钳位 (上限 / 非法关阀 / 占空比钳位)
 *  @note  无钳位时 htime > period 会使 cc0_start 下溢成大数 → 底层直接 return
 *         → 阀完全不动作且无任何提示 (静默失效)。 */
static void Psw_ValActClamp(uint16_t *period, uint16_t *htime)
{
    if (*period > RTE_PERIOD_MAX) { *period = RTE_PERIOD_MAX; }   /* 周期上限   */
    if (*period == 0u)            { *period = 1u; *htime = 0u; }  /* 非法 → 关阀 */
    else if (*htime > *period)    { *htime  = *period; }          /* 占空比钳位 */
}

/* ========================================================================== */
/*  全局/静态状态                                                              */
/* ========================================================================== */

/* ========================================================================== */
/*  内部辅助函数                                                               */
/* ========================================================================== */

/** @brief 进气阀 PWM (后段高, LOW→HIGH): CC0=period-htime, CC1=period */
static void Psw_InValAct(valve_id_t vid, uint16_t period, uint16_t htime)
{
    if (vid >= VALVE_NUM_TOTAL) return;

    Psw_ValActClamp(&period, &htime);

    if (htime == 0u)
    {
        Bts724g_NotifyValveActive(vid, false);
        Bts724g_SetValve(vid, false);
        return;
    }

    uint16_t pwm_period = RTE_TO_PWM_STEPS(period);
    uint16_t pwm_htime  = RTE_TO_PWM_STEPS(htime);
    uint16_t cc0_start  = pwm_period - pwm_htime;

    Bts724g_NotifyValveActive(vid, true);
    /* 运行中动态调参: 参数立即生效 + 保持相位连续。
     * 参数未变化由内部幂等短路直接返回, 不会反复重置周期。 */
    ValvePwm_SetDutyLive(vid, pwm_period, cc0_start, pwm_htime);
}

/** @brief 排气阀 PWM (前段高, HIGH→LOW): CC0=OFFSET(0, 周期起点), CC1=htime (21口/22口 排气阀同样适用) */
static void Psw_OutValAct(valve_id_t vid, uint16_t period, uint16_t htime)
{
    if (vid >= VALVE_NUM_TOTAL) return;

    Psw_ValActClamp(&period, &htime);

    if (htime == 0u)
    {
        Bts724g_NotifyValveActive(vid, false);
        Bts724g_SetValve(vid, false);
        return;
    }

    uint16_t pwm_period = RTE_TO_PWM_STEPS(period);
    uint16_t pwm_htime  = RTE_TO_PWM_STEPS(htime);

    Bts724g_NotifyValveActive(vid, true);
    /* 同 Psw_InValAct: 立即生效 + 相位连续 (参数未变由幂等短路挡掉) */
    ValvePwm_SetDutyLive(vid, pwm_period, PWM_CC0_OFFSET, pwm_htime);
}

/* ========================================================================== */
/*  22口 阀低边开关 (RTE form PSW 接口, 由 ASW 层调用)                          */
/*                                                                            */
/*    In  → P2.0 (原 ASRF / FA_ASR 低边): 22口进气阀 / TR_ASR / 21口 /          */
/*            RXI/RXO/LXO/LXI 共用同一低边                                      */
/*    Out → P6.2 (原 ASRR / DA_ASR 低边): 22口排气阀                            */
/*                                                                            */
/*  低边权限归 ASW (诊断已不切换; 上电由 Bts724g_Init() 打开, ref=1):          */
/*    (1) → ref+1 (≤255), 低边闭合                                            */
/*    (0) → ref-1, 减到 0 时低边断开                                          */
/*  幂等保护: 仅状态变化才动引用计数, 防 ASW 周期重复调用导致漂移/溢出。       */
/*  ⚠ hold 初值 = true, 与上电即开的实际状态对齐 —— 若为 false, ASW 首次置 0   */
/*    会被幂等短路吞掉, 低边将永远关不掉 (2026-10-08 修)。                     */
/* ========================================================================== */

static bool s_in22_lowside_hold  = true;    /* ASW 侧保持请求 (P2.0), 初值对齐上电即开 */
static bool s_out22_lowside_hold = true;    /* ASW 侧保持请求 (P6.2), 初值对齐上电即开 */

void InLowSideSwX4_16(uint32_t sw)
{
    const bool on = (sw != 0u);
    if (on == s_in22_lowside_hold) return;      /* 幂等 */
    s_in22_lowside_hold = on;
    Gpio_ASRFLowSideEnable(on ? 1u : 0u);
}

uint32_t InLowSideStX4_16(void) { return Gpio_ASRFLowSideSt(); }   /* 硬件回读 */

void OutLowSideSwX2_16(uint8_t sw)
{
    const bool on = (sw != 0u);
    if (on == s_out22_lowside_hold) return;     /* 幂等 */
    s_out22_lowside_hold = on;
    Gpio_ASRRLowSideEnable(on ? 1u : 0u);
}

uint32_t OutLowSideStX2_16(void) { return Gpio_ASRRLowSideSt(); }  /* 硬件回读 */

/* ---- 旧名 (RTE.h 已删声明, PSWdebug 波形测试仍在用): 复用同一 hold, 避免两套状态 ---- */
void     ASRFLowSideSw(uint32_t sw)       { InLowSideSwX4_16(sw); }
uint32_t ASRFLowSideSt(void)              { return InLowSideStX4_16(); }
void     ASRRLowSideSw(uint8_t sw)        { OutLowSideSwX2_16(sw); }
uint32_t ASRRLowSideSt(void)             { return OutLowSideStX2_16(); }

/* ---- 引用计数包装 (bts724g.c 仅 Bts724g_Init() 上电打开一次时用) ---- */
void     ASRFLowSideEnable(uint8_t e)     { Gpio_ASRFLowSideEnable(e); }
void     ASRRLowSideEnable(uint8_t e)     { Gpio_ASRRLowSideEnable(e); }

/* ========================================================================== */
/*  ABS 阀控制                                                                 */
/* ========================================================================== */

void InValActFL(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_LFI, period, htime); }
void OutValActFL(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_LFO, period, htime); }

void InValActFR(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_RFI, period, htime); }
void OutValActFR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_RFO, period, htime); }

void InValActRL(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_LRI, period, htime); }
void OutValActRL(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_LRO, period, htime); }

void InValActRR(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_RRI, period, htime); }
void OutValActRR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_RRO, period, htime); }

void InValActXL(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_LXI, period, htime); }
void OutValActXL(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_LXO, period, htime); }

void InValActXR(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_RXI, period, htime); }
void OutValActXR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_RXO, period, htime); }

/* ========================================================================== */
/*  21口 / 22口 阀控制 (RTE 2026-09-11 改为按接插件引脚命名)                     */
/*                                                                            */
/*    X3_7  (21口进气, U12 IN1 / 原 TRFIN)  InValActX3_7                     */
/*    X3_10 (21口排气, U12 IN2 / 原 TRFOUT) OutValActX3_10                    */
/*    X4_16 (22口进气, U13 IN1 / 原 FA_ASR) InValActX4_16                     */
/*    X2_16 (22口排气, U13 IN2 / 原 DA_ASR) OutValActX2_16                    */
/*  对应故障字段: RTEfErrX3_7InValve / RTEfErrX3_10OutValve /                   */
/*                RTEfErrX4_16InValve / RTEfErrX2_16OutValve (psw_data.c 写入)  */
/* ========================================================================== */

void InValActX3_7(uint16_t period, uint16_t htime)  { Psw_InValAct(VALVE_21IN,  period, htime); }
void OutValActX3_10(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_21OUT, period, htime); }

void InValActX4_16(uint16_t period, uint16_t htime)  { Psw_InValAct(VALVE_22IN,  period, htime); }
void OutValActX2_16(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_22OUT, period, htime); }

// /* ---- 旧名兼容 (RTE.h 已删声明; 未重编的 COM/ASW 库与 PSWdebug 波形测试仍引用) ----
//  * TODO: ASW/COM 层重编后可删除本组 */
// void InValAct21(uint16_t period, uint16_t htime)  { InValActX3_7(period, htime); }
// void OutValAct21(uint16_t period, uint16_t htime) { OutValActX3_10(period, htime); }
// void InValAct22(uint16_t period, uint16_t htime)  { InValActX4_16(period, htime); }
// void OutValAct22(uint16_t period, uint16_t htime) { OutValActX2_16(period, htime); }

/* TR_ASR: 新 RTE 未导出该接口, 保留实现 (防止未重编的 COM/ASW 侧链接失败) */
void ASRValActX(uint16_t period, uint16_t htime)  { Psw_OutValAct(VALVE_TR_ASR, period, htime); }

/* ========================================================================== */
/*  继电器 / UB / 接插件引脚 (委托 gpio.c 实现)                                */
/* ========================================================================== */

void RelayCtrl(uint8_t state)       { Gpio_RelayCtrl(state); }
void UBCtrl(uint8_t state)          { Gpio_UBCtrl(state); }
void X1_2_staSet(uint8_t setValue)  { Gpio_HSALampCtrl(setValue); }
void X1_13_staSet(uint8_t setValue) { Gpio_ASRLampCtrl(setValue); }
void X1_15_staSet(uint8_t setValue) { Gpio_ABSWLampCtrl(setValue); }
int8_t Xn_pin_StaGet(uint8_t xNum, uint8_t xPin) { return Gpio_Xn_pin_StaGet(xNum, xPin); }

/* ========================================================================== */
/*  版本信息                                                                   */
/* ========================================================================== */

/* RTEPSW_Version 属 RTE.h 的 "form PSW" 变量 → 由 PSW 库侧定义 (RTE.c 中不再定义, 避免符号重复)
 * 格式: BCD {层标识01, 年低位, 月, 日, 当天第N次}, 如 {0x01,0x26,0x09,0x23,0x01} = 2026-09-23 第1次
 * ⚠ 静态初值与 RtePsw_VersionInit() 必须保持一致 (Init 在 cmn.c 启动时调用, 会覆盖静态初值) */
uint8_t RTEPSW_Version[5] = {0x01u, 0x26u, 0x10u, 0x08u, 0x01u};

/**
 * @brief 初始化 RTEPSW_Version (BCD 码 {底层01, 年低位, 月, 日, 修改当天版本号})
 * @note  定义在本文件 (PSW 库侧); 每次修改底层工程按当天日期更新,
 *        [4] 为当天第 N 次修改 (次日归 0x01)。
 */
void RtePsw_VersionInit(void)
{
    RTEPSW_Version[0] = 0x01u;   /* 底层01*/
    RTEPSW_Version[1] = 0x26u;   /* 年低位 26 → 2026 */
    RTEPSW_Version[2] = 0x10u;   /* 月  */
    RTEPSW_Version[3] = 0x08u;   /* 日  */
    RTEPSW_Version[4] = 0x01u;   /* 当天第 1 次修改 */
}

uint8_t psw_version_get(uint8_t *verStr, uint8_t bufLen)
{
    if (bufLen < 8u) return 0u;
    const char version[] = "RBS_PSW_V1.0.0";
    uint8_t len = (uint8_t)sizeof(version);
    if (len > bufLen) len = bufLen;
    (void)memcpy(verStr, version, len);
    return len;
}

/* ========================================================================== */
/*  EEPROM 读写 (5 分区: ERR_CODE / ASW / PSW / COM / BOOT)                    */
/*                                                                            */
/*  故障上报下沉到 eeprom.c Eeprom_Write/Read, 本层纯薄封装                     */
/* ========================================================================== */

uint32_t ERR_CODE_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_ERR_CODE_BASE, wAddr, data, len, EEPROM_ERR_CODE_SIZE); }
uint32_t ERR_CODE_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_ERR_CODE_BASE, rAddr, buf, len, EEPROM_ERR_CODE_SIZE); }

uint32_t ASW_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_ASW_BASE, wAddr, data, len, EEPROM_ASW_CFG_SIZE); }
uint32_t ASW_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_ASW_BASE, rAddr, buf, len, EEPROM_ASW_CFG_SIZE); }

uint32_t PSW_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_PSW_BASE, wAddr, data, len, EEPROM_PSW_SIZE); }
uint32_t PSW_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_PSW_BASE, rAddr, buf, len, EEPROM_PSW_SIZE); }

uint32_t COM_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_COM_BASE, wAddr, data, len, EEPROM_COM_SIZE); }
uint32_t COM_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_COM_BASE, rAddr, buf, len, EEPROM_COM_SIZE); }

uint32_t BOOT_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_BOOT_BASE, wAddr, data, len, EEPROM_BOOT_SIZE); }
uint32_t BOOT_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_BOOT_BASE, rAddr, buf, len, EEPROM_BOOT_SIZE); }

/* ========================================================================== */
/*  SCC3000 (ESC 传感器) 桩函数 — 已废弃                                        */
/*                                                                            */
/*  SCC3000/IMU 驱动不在本工程范围, ESC 传感器数据 (YawRate/LatAcc/LongiAcc 等) */
/*  由外部模块负责。此处仅为满足 COM 层链接提供桩函数。                           */
/* ========================================================================== */

/** @brief ESC 标定请求 (桩: 无实际操作) */
void SCC3000_calibration_req(void)
{
    (void)0;
}

/** @brief ESC 反标定请求 (桩: 无实际操作) */
void SCC3000_UnCalibration_req(void)
{
    (void)0;
}

/** @brief ESC 标定结果反馈 (桩: 恒返回 0 = 无标定流程) */
uint8_t SCC3000_calibration_resp(void)
{
    return 0u;
}

/** @brief 获取 ESC 是否故障 (桩: 恒返回 0 = 无故障) */
uint8_t SCC3000_Err(void)
{
    return 0u;
}

/** @brief 获取 ECU 类型 (桩: 恒返回 0 = 驾驶室版, 待后续按配置实现) */
uint8_t get_ECU_type(void)
{
    return 0u;
}
