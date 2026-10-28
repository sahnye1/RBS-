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
 *            22口 → X4_16/X2_16
 */

#include "RTE.h"
#include "bts724g.h"
#include "valve_pwm.h"
#include "gpio_pwm.h"
#include "eeprom.h"
#include "gpio.h"
#include "psw_data.h"
#include <string.h>
#include "rte_psw.h"

/* ========================================================================== */
/*  1. 常量与宏                                                                 */
/* ========================================================================== */

/* RTE 0.1ms → ValvePwm 10μs 步数 */
#define RTE_TO_PWM_STEPS(v)      ((uint16_t)((v) * 10u))

/* RTE 层周期上限 (单位 0.1ms): 5000 = 500ms */
#define RTE_PERIOD_MAX           5000u

/* ========================================================================== */
/*  2. 内部辅助函数                                                             */
/* ========================================================================== */

/** @brief RTE 阀控参数钳位 (上限 / 非法关阀 / 占空比钳位)
 *  @note  无钳位时 htime > period 会使 cc0_start 下溢成大数 → 底层直接 return
 *         → 阀完全不动作且无任何提示 (静默失效)。
 */
static void Psw_ValActClamp(uint16_t *period, uint16_t *htime)
{
    if (*period > RTE_PERIOD_MAX) { *period = RTE_PERIOD_MAX; }   /* 周期上限   */
    if (*period == 0u)            { *period = 1u; *htime = 0u; }  /* 非法 → 关阀 */
    else if (*htime > *period)    { *htime  = *period; }          /* 占空比钳位 */
}

/** @brief 进气阀 PWM (后段高, LOW→HIGH): CC0 = period-htime, CC1 = period
 *  @param vid    阀 ID / @param period 周期 (0.1ms) / @param htime 高电平时间 (0.1ms)
 */
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

    Bts724g_NotifyValveActive(vid, true);

    if (ValvePwm_IsPwmValve(vid))
    {
        /* 硬件 PWM (10μs 步): 进气阀后段高 cc0 = period - htime */
        uint16_t pwm_period = RTE_TO_PWM_STEPS(period);
        uint16_t pwm_htime  = RTE_TO_PWM_STEPS(htime);
        ValvePwm_SetDutyLive(vid, pwm_period, (uint16_t)(pwm_period - pwm_htime), pwm_htime);
    }
    else
    {
        /* GPIO 软件 PWM (0.1ms 节拍): 进气阀后段高 cc0 = period - htime */
        GpioPwm_SetDuty(vid, period, (uint16_t)(period - htime), htime);
    }
}

/** @brief 排气阀 PWM (前段高, HIGH→LOW): CC0 = PWM_CC0_OFFSET(0, 周期起点), CC1 = htime
 *  @param vid    阀 ID / @param period 周期 (0.1ms) / @param htime 高电平时间 (0.1ms)
 */
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

    Bts724g_NotifyValveActive(vid, true);

    if (ValvePwm_IsPwmValve(vid))
    {
        /* 硬件 PWM (10μs 步): 排气阀前段高 cc0 = 0 (周期起点) */
        uint16_t pwm_period = RTE_TO_PWM_STEPS(period);
        uint16_t pwm_htime  = RTE_TO_PWM_STEPS(htime);
        ValvePwm_SetDutyLive(vid, pwm_period, PWM_CC0_OFFSET, pwm_htime);
    }
    else
    {
        /* GPIO 软件 PWM (0.1ms 节拍): 排气阀前段高 cc0 = 0 */
        GpioPwm_SetDuty(vid, period, PWM_CC0_OFFSET, htime);
    }
}

/* ========================================================================== */
/*  3. 22口 阀低边开关 (RTE form PSW 接口, 由 ASW 层调用)                       */
/*                                                                            */
/*    In  → P2.0 (原 ASRF / FA_ASR 低边): 22口进气阀 / TR_ASR / 21口 /          */
/*            RXI/RXO/LXO/LXI 共用同一低边                                      */
/*    Out → P6.2 (原 ASRR / DA_ASR 低边): 22口排气阀                            */
/*                                                                            */
/*  低边开关: 上电由 Bts724g_Init() 直写打开一次, 之后全权交由 ASW 控制:          */
/*    (1) → 直写闭合                                                            */
/*    (0) → 直写断开                                                            */
/*  本接口直写 GPIO, 无引用计数/无幂等缓存, 每次调用立即生效。                     */
/* ========================================================================== */

/** @brief 前桥 ASR 低边开关 (P2.0): 1=闭合 0=断开, 直写无缓存 */
void InLowSideSwX4_16(uint32_t sw) { Gpio_ASRFLowSideSw((sw != 0u) ? 1u : 0u); }

/** @brief 前桥 ASR 低边状态回读 (硬件反馈): 1=闭合 0=断开 */
uint32_t InLowSideStX4_16(void) { return Gpio_ASRFLowSideSt(); }

/** @brief 后桥 ASR 低边开关 (P6.2): 1=闭合 0=断开, 直写无缓存 */
void OutLowSideSwX2_16(uint8_t sw) { Gpio_ASRRLowSideSw((sw != 0u) ? 1u : 0u); }

/** @brief 后桥 ASR 低边状态回读 (硬件反馈): 1=闭合 0=断开 */
uint32_t OutLowSideStX2_16(void) { return Gpio_ASRRLowSideSt(); }

/* ========================================================================== */
/*  4. ABS 阀控制 (6 轮 × 进/排气)                                              */
/*                                                                            */
/*  命名: In/Out = 进气/排气; FL/FR/RL/RR = 左前/右前/左后/右后;              */
/*        XL/XR = 挂车左/右                                                     */
/*  In  → Psw_InValAct  (后段高) / Out → Psw_OutValAct (前段高)                 */
/* ========================================================================== */

/** @brief 左前进气阀 PWM (后段高) */      void InValActFL(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_LFI, period, htime); }
/** @brief 左前排排气阀 PWM (前段高) */    void OutValActFL(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_LFO, period, htime); }

/** @brief 右前进气阀 PWM (后段高) */      void InValActFR(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_RFI, period, htime); }
/** @brief 右前排排气阀 PWM (前段高) */    void OutValActFR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_RFO, period, htime); }

/** @brief 左后进气阀 PWM (后段高) */      void InValActRL(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_LRI, period, htime); }
/** @brief 左后排排气阀 PWM (前段高) */    void OutValActRL(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_LRO, period, htime); }

/** @brief 右后进气阀 PWM (后段高) */      void InValActRR(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_RRI, period, htime); }
/** @brief 右后排排气阀 PWM (前段高) */    void OutValActRR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_RRO, period, htime); }

/** @brief 挂车左进气阀 PWM (后段高) */    void InValActXL(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_LXI, period, htime); }
/** @brief 挂车左排排气阀 PWM (前段高) */  void OutValActXL(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_LXO, period, htime); }

/** @brief 挂车右进气阀 PWM (后段高) */    void InValActXR(uint16_t period, uint16_t htime) { Psw_InValAct(VALVE_RXI, period, htime); }
/** @brief 挂车右排排气阀 PWM (前段高) */  void OutValActXR(uint16_t period, uint16_t htime) { Psw_OutValAct(VALVE_RXO, period, htime); }

/* ========================================================================== */
/*  5. 21口 / 22口 阀控制 (RTE 2026-09-11 改为按接插件引脚命名)                 */
/*                                                                            */
/*    X3_7  (21口进气, U12 IN1 / 原 TRFIN)  InValActX3_7                     */
/*    X3_10 (21口排气, U12 IN2 / 原 TRFOUT) OutValActX3_10                    */
/*    X4_16 (22口进气, U13 IN1 / 原 FA_ASR) InValActX4_16                     */
/*    X2_16 (22口排气, U13 IN2 / 原 DA_ASR) OutValActX2_16                    */
/*  对应故障字段: RTEfErrX3_7InValve / RTEfErrX3_10OutValve /                   */
/*                RTEfErrX4_16InValve / RTEfErrX2_16OutValve (psw_data.c 写入)  */
/*  ⚠ 2026-10-20: 进气/排气波形已对调 (进气→前段高, 排气→后段高)               */
/* ========================================================================== */

/** @brief 21口进气阀 PWM (2026-10-20 对调为前段高) */
void InValActX3_7(uint16_t period, uint16_t htime)
{
    Psw_OutValAct(VALVE_21IN, period, htime);      /* 2026-10-20: 进气阀波形对调 (后段高→前段高) */
}

/** @brief 21口排气阀 PWM (2026-10-20 对调为后段高) */
void OutValActX3_10(uint16_t period, uint16_t htime)
{
    Psw_InValAct(VALVE_21OUT, period, htime);      /* 2026-10-20: 排气阀波形对调 (前段高→后段高) */
}

/** @brief 22口进气阀 PWM (2026-10-20 对调为前段高) */
void InValActX4_16(uint16_t period, uint16_t htime)
{
    Psw_OutValAct(VALVE_22IN, period, htime);      /* 2026-10-20: 进气阀波形对调 (后段高→前段高) */
}

/** @brief 22口排气阀 PWM (2026-10-20 对调为后段高) */
void OutValActX2_16(uint16_t period, uint16_t htime)
{
    Psw_InValAct(VALVE_22OUT, period, htime);      /* 2026-10-20: 排气阀波形对调 (前段高→后段高) */
}

/* ========================================================================== */
/*  6. 继电器 / UB / 传感器使能 / 指示灯 / 接插件引脚 (委托 gpio.c 实现)         */
/* ========================================================================== */

/** @brief 继电器控制 (委托 Gpio_RelayCtrl): 1=闭合 0=断开 */
void RelayCtrl(uint8_t state)       { Gpio_RelayCtrl(state); }

/** @brief UB 电压输出控制 (委托 Gpio_UBCtrl): 1=开启 0=关闭 */
void UBCtrl(uint8_t state)          { Gpio_UBCtrl(state); }

/** @brief 5V 压力传感器使能 PRE5V_EN (P18.2): 1=打开 0=关闭 */
void Pre5vEnCtrl(uint8_t state)     { Gpio_Pre5vEnCtrl(state); }

/** @brief X1-2 指示灯 (HSA): 1=亮 0=灭 */
void X1_2_staSet(uint8_t setValue)  { Gpio_HSALampCtrl(setValue); }

/** @brief X1-13 指示灯 (ASR): 1=亮 0=灭 */
void X1_13_staSet(uint8_t setValue) { Gpio_ASRLampCtrl(setValue); }

/** @brief X1-15 指示灯 (ABS): 1=亮 0=灭 */
void X1_15_staSet(uint8_t setValue) { Gpio_ABSWLampCtrl(setValue); }

/** @brief 接插件引脚状态读取: 1=接地 0=悬空 -1=参数非法 */
int8_t Xn_pin_StaGet(uint8_t xNum, uint8_t xPin) { return Gpio_Xn_pin_StaGet(xNum, xPin); }

/* ========================================================================== */
/*  7. 版本信息                                                                 */
/* ========================================================================== */

/* RTEPSW_Version 属 RTE.h 的 "form PSW" 变量 → 由 PSW 库侧定义 (RTE.c 中不再定义, 避免符号重复)
 * 格式: BCD {层标识01, 年低位, 月, 日, 当天第N次}, 如 {0x01,0x26,0x09,0x23,0x01} = 2026-09-23 第1次
 * ⚠ 静态初值与 RtePsw_VersionInit() 必须保持一致 (Init 在 cmn.c 启动时调用, 会覆盖静态初值) */
uint8_t RTEPSW_Version[5] = {0x01u, 0x26u, 0x10u, 0x27u, 0x01u};

/** @brief 初始化 RTEPSW_Version (BCD 码 {底层01, 年低位, 月, 日, 当天第N次})
 *  @note  定义在本文件 (PSW 库侧); 每次修改底层工程按当天日期更新,
 *         [4] 为当天第 N 次修改 (次日归 0x01)。
 */
void RtePsw_VersionInit(void)
{
    RTEPSW_Version[0] = 0x01u;   /* 底层01*/
    RTEPSW_Version[1] = 0x26u;   /* 年低位 26 → 2026 */
    RTEPSW_Version[2] = 0x10u;   /* 月  */
    RTEPSW_Version[3] = 0x27u;   /* 日  */
    RTEPSW_Version[4] = 0x01u;   /* 当天第 1 次修改 */
}

/** @brief 读取 PSW 版本字符串
 *  @param verStr 输出缓冲 / @param bufLen 缓冲长度 (需 ≥ 8)
 *  @return 实际写入长度 (bufLen 不足时返回 0)
 */
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
/*  8. EEPROM 读写 (5 分区: ERR_CODE / ASW / PSW / COM / BOOT)                  */
/*                                                                            */
/*  故障上报下沉到 eeprom.c Eeprom_Write/Read, 本层纯薄封装                     */
/* ========================================================================== */

/** @brief ERR_CODE 分区写 */ uint32_t ERR_CODE_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_ERR_CODE_BASE, wAddr, data, len, EEPROM_ERR_CODE_SIZE); }
/** @brief ERR_CODE 分区读 */ uint32_t ERR_CODE_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_ERR_CODE_BASE, rAddr, buf, len, EEPROM_ERR_CODE_SIZE); }

/** @brief ASW 配置分区写 */ uint32_t ASW_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_ASW_BASE, wAddr, data, len, EEPROM_ASW_CFG_SIZE); }
/** @brief ASW 配置分区读 */ uint32_t ASW_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_ASW_BASE, rAddr, buf, len, EEPROM_ASW_CFG_SIZE); }

/** @brief PSW 配置分区写 */ uint32_t PSW_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_PSW_BASE, wAddr, data, len, EEPROM_PSW_SIZE); }
/** @brief PSW 配置分区读 */ uint32_t PSW_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_PSW_BASE, rAddr, buf, len, EEPROM_PSW_SIZE); }

/** @brief COM 配置分区写 */ uint32_t COM_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_COM_BASE, wAddr, data, len, EEPROM_COM_SIZE); }
/** @brief COM 配置分区读 */ uint32_t COM_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_COM_BASE, rAddr, buf, len, EEPROM_COM_SIZE); }

/** @brief BOOT 配置分区写 */ uint32_t BOOT_cfg_write(uint32_t wAddr, uint8_t *data, uint32_t len)
    { return Eeprom_Write(EEPROM_BOOT_BASE, wAddr, data, len, EEPROM_BOOT_SIZE); }
/** @brief BOOT 配置分区读 */ uint32_t BOOT_cfg_read(uint32_t rAddr, uint8_t *buf, uint32_t len)
    { return Eeprom_Read(EEPROM_BOOT_BASE, rAddr, buf, len, EEPROM_BOOT_SIZE); }

/* ========================================================================== */
/*  9. SCC3000 (ESC 传感器) 桩函数 — 已废弃                                     */
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

/* ========================================================================== */
/*  10. ECU 类型                                                                */
/* ========================================================================== */

/** @brief 获取 ECU 类型 (桩: 恒返回 0 = 驾驶室版, 待后续按配置实现) */
uint8_t get_ECU_type(void)
{
    return 0u;
}
