/**
 * @file    gpio.c
 * @brief   GPIO 通用输出/输入控制 (继电器/UB/传感器使能/接插件/指示灯/ASR低边开关)
 *
 * @note    继电器 VPOWER 检测: Gpio_RelayCtrl() 动作后, 由 Gpio_RelayTick() (每 10ms)
 *          逐拍读 VPOWER — 达标(ON≥20V / OFF≤2V)当拍定案; 8 拍(80ms)未达标则
 *          连续 3 拍异常判故障。
 */

#include "gpio.h"
#include "adc.h"
#include "psw_data.h"
#include "cmn.h"       /* g_skipHardWD / g_skipSoftWD (看门狗跳过标记) */

/* ========================================================================== */
/*  1. 引脚配置表                                                               */
/*                                                                            */
/*  字段: portReg, pinNum, outVal, driveMode, hsiom, intEdge, intMask,         */
/*        vtrip, slewRate, driveSel, vregEn, ibufMode, vtripSel, vrefSel, vohSel */
/* ========================================================================== */
static const cy_stc_gpio_pin_prt_config_t s_gpio_pin_cfg[] = {

    /* ---- 输出 ---- */
    {RELAY_GPIO_PORT, RELAY_GPIO_PIN, 1ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   /* 继电器 (P14.3), 上电闭合 */
    {UB_GPIO_PORT,    UB_GPIO_PIN,    1ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   /* UB 供电 (P13.6), 上电开启 */

    /* ---- 传感器供电使能, 上电打开 (底层初始化时置高) ---- */
    {PRE5V_EN_GPIO_PORT, PRE5V_EN_GPIO_PIN, 1ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* PRE5V_EN 压力传感器使能 (P18.2), 高=打开, 上电默认开 */

    /* ---- 指示灯, 上电灭 ---- */
    {ABS_WLAMP_GPIO_PORT, ABS_WLAMP_GPIO_PIN, 0ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* ABS 灯 (P18.0) */
    {HSA_LAMP_GPIO_PORT,  HSA_LAMP_GPIO_PIN,  0ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* HSA 灯 (P18.4) */
    {ASR_LAMP_GPIO_PORT,  ASR_LAMP_GPIO_PIN,  0ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* ASR 灯 (P11.2) */

    /* ---- ASR 低边开关, 上电关断 ---- */
    {ASR_F_LOWSIDE_CTRL_PORT, ASR_F_LOWSIDE_CTRL_PIN, 0ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* FA 低边 (P2.0) */
    {ASR_R_LOWSIDE_CTRL_PORT, ASR_R_LOWSIDE_CTRL_PIN, 0ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* DA 低边 (P6.2) */

    /* ---- 硬件看门狗喂狗 (P5.0): wd_feed() 翻转, AC 耦合驱动 QFAR2 ---- */
    {WD_HW_GPIO_PORT, WD_HW_GPIO_PIN, 0ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

    /* ---- 输入: 接插件开关 (High-Z) ---- */
    {X3_4_GPIO_PORT, X3_4_GPIO_PIN, 0ul, CY_GPIO_DM_HIGHZ,        HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* X3-4 (P18.1) */
    {X1_6_GPIO_PORT, X1_6_GPIO_PIN, 0ul, CY_GPIO_DM_HIGHZ,        HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* X1-6 (P11.0) */
    {X1_5_GPIO_PORT, X1_5_GPIO_PIN, 0ul, CY_GPIO_DM_HIGHZ,        HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /* X1-5 (P3.0)  */
};

/* ========================================================================== */
/*  2. 初始化                                                                   */
/*                                                                            */
/*  Cy_GPIO_Multi_Pin_Init 遍历数组, 内部逐条调用 Cy_GPIO_Pin_Init。            */
/* ========================================================================== */

/** @brief GPIO 引脚初始化 — 按配置表批量设置 (继电器/UB/传感器使能/指示灯/接插件, 上电默认状态)
 */
void Gpio_Init(void)
{
    Cy_GPIO_Multi_Pin_Init(s_gpio_pin_cfg,
                           (uint32_t)(sizeof(s_gpio_pin_cfg) / sizeof(s_gpio_pin_cfg[0])));
}

/* ========================================================================== */
/*  3. 看门狗                                                                   */
/*                                                                            */
/*  硬狗: P5.0 翻转, 经 AC 耦合驱动 QFAR2 (与 QFAR1 串联保持继电器闭合)          */
/*  软狗: 芯片内部 WDT 清零 (Upper Limit 由 SoftWD_init 设定)                   */
/*  跳过标记: g_skipHardWD / g_skipSoftWD = 1 时停喂 (PSWdebug 看门狗超时测试用) */
/* ========================================================================== */

/** @brief 双看门狗喂狗 — 硬狗 (P5.0 翻转) + 软狗 (Cy_WDT_ClearWatchdog)
 *  @note  每 10ms 主循环调用一次; g_skipHardWD / g_skipSoftWD = 1 时对应狗停喂
 *         (2026-10-20 从 wd_feed.h / cmn.c 统一到本文件维护)
 */
void wd_feed(void)
{
    if (g_skipHardWD == 0u)
    {
        Cy_GPIO_Inv(WD_HW_GPIO_PORT, WD_HW_GPIO_PIN);   /* 硬件看门狗: P5.0 翻转 */
    }
    if (g_skipSoftWD == 0u)
    {
        Cy_WDT_ClearWatchdog();                          /* 软件看门狗: WDT 清零 */
    }
}

/* ========================================================================== */
/*  4. 电源使能 (PRE5V_EN)                                                      */
/*                                                                            */
/*  P18.2 输出高 = 打开 (压力传感器 5V 供电); 上电由配置表 outVal=1 置高         */
/* ========================================================================== */

/** @brief 压力传感器 5V 使能 PRE5V_EN (P18.2): 1=打开 0=关闭
 *  @note  上电默认打开 (Gpio_Init 配置表 outVal=1); 应用层需要时可用本函数关断/重新打开
 */
void Gpio_Pre5vEnCtrl(uint8_t on)
{
    if (!GPIO_IS_VALID(PRE5V_EN_GPIO_PORT, PRE5V_EN_GPIO_PIN)) return;
    Cy_GPIO_Write(PRE5V_EN_GPIO_PORT, PRE5V_EN_GPIO_PIN, (on != 0u) ? 1u : 0u);
}

/* ========================================================================== */
/*  5. 数字输入                                                                 */
/* ========================================================================== */

/** @brief 地码输入读取 (P8.0, 低有效: 0=接地 1=悬空)
 *  @return 0 = 接地, 1 = 悬空 (引脚无效时返回 1)
 */
uint8_t Gpio_DiMaRead(void)
{
    if (!GPIO_IS_VALID(DIMA_GPIO_PORT, DIMA_GPIO_PIN)) return 1u;
    return (Cy_GPIO_Read(DIMA_GPIO_PORT, DIMA_GPIO_PIN) == 0u) ? 0u : 1u;
}

/* ========================================================================== */
/*  6. 测试标记                                                                 */
/*                                                                            */
/*  输出口: P22.0 / 电路 LROCN (后桥左排阀, 接插件 X2-5)                        */
/*                                                                            */
/*  测试用例需要在接插件上量到高低电平来测时序 (10ms 节拍 / EEPROM 读写耗时 /      */
/*  看门狗超时), 故首次调用时把该阀引脚由 TCPWM 切为 GPIO 推挽输出; 切走后该阀      */
/*  不再受 PWM 控制 (TCPWM 内部照常运行, 只是到不了引脚), 由本函数直接驱动。       */
/*  ⚠ 测试期间这一路会按标记信号通断, 该阀不要接负载; 不切回, 复位后恢复默认。      */
/* ========================================================================== */

#define TEST_MARK_PORT   GPIO_PRT22
#define TEST_MARK_PIN    0u
#define TEST_MARK_HSIOM  P22_0_GPIO

static const cy_stc_gpio_pin_config_t s_test_mark_cfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,   /* 推挽输出 */
    .hsiom     = TEST_MARK_HSIOM,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/** @brief 测试计时标记输出 (P22.0) — 首次调用把该引脚从 TCPWM 切为 GPIO 推挽
 *  @param state 0=低 1=高
 *  @note  仅测试用; 该阀测试期间不可接负载, 复位后恢复默认
 */
void TestMark_Ctrl(uint8_t state)
{
    static uint8_t s_test_mark_inited = 0u;

    if (s_test_mark_inited == 0u)
    {
        Cy_GPIO_Pin_Init(TEST_MARK_PORT, TEST_MARK_PIN, &s_test_mark_cfg);   /* 首次: 脱离 TCPWM, 切为 GPIO */
        s_test_mark_inited = 1u;
    }

    Cy_GPIO_Write(TEST_MARK_PORT, TEST_MARK_PIN, (state != 0u) ? 1u : 0u);
}

/* ========================================================================== */
/*  7. 继电器控制                                                               */
/*                                                                            */
/*  动作后 等 VPOWER 到位, 非阻塞; 两方向不对称(证据强度不同):                  */
/*  ON  → 任一拍 VPOWER ≥20V 即命中(触点已闭合物证) → 当拍判正常, 不消抖         */
/*       → 8 拍(80ms) 仍未达 20V 视为异常 → 连续 3 拍异常才判开路               */
/*  OFF → 任一拍 VPOWER ≤2V  即命中(触点已断物证) → 当拍判正常, 不消抖           */
/*       → 8 拍(80ms) 仍未降到 2V 视为异常 → 连续 3 拍异常才判闭合              */
/*  ⚠ 正常方向"看到即定案": VPOWER 达标是物理事实, 无需再等时间。                */
/*  ⚠ 异常方向必须消抖: "还没吸合完"与"继电器已坏"物理上无法区分, 只能超时+消抖。 */
/*  ⚠ 继电器故障(PSWErrRelay)与 VPOWER 电压报警(PSWErrVpwrLo/Hi, ADC 18V/36V)  */
/*    取值同源(都读 VPOWER)但为两条独立判断路, 本例只写 PSWErrRelay。            */
/* ========================================================================== */

#define RELAY_SETTLE_TICKS      8u      /* 等待上限: 8 拍 × 10ms = 80ms (吸合 + 建立), 兼作故障超时 */
#define RELAY_DEBOUNCE_TICKS    3u      /* 超时后连续 3 拍 (30ms) 异常才判故障 */
#define RELAY_ON_VPWR_MIN       200u    /* 20.0V */
#define RELAY_OFF_VPWR_MAX       20u    /*  2.0V */

static uint8_t s_relay_pending   = 0u;  /* 检测进行中 */
static uint8_t s_relay_target    = 0u;  /* 目标: 1=闭合 0=断开 */
static uint8_t s_relay_ticks     = 0u;  /* 等待上限倒计时 */
static uint8_t s_relay_err_ticks = 0u;  /* 连续异常计数 */

/** @brief 继电器开关控制 — 立即切换 GPIO, 并由 Gpio_RelayTick 做到位判定
 *  @param state 0=断开 1=闭合
 */
void Gpio_RelayCtrl(uint8_t state)
{
    if (state == 0u)
    {
        Cy_GPIO_Clr(RELAY_GPIO_PORT, RELAY_GPIO_PIN);
    }
    else
    {
        Cy_GPIO_Set(RELAY_GPIO_PORT, RELAY_GPIO_PIN);
    }

    s_relay_pending   = 1u;
    s_relay_target    = (state != 0u) ? 1u : 0u;
    s_relay_ticks     = RELAY_SETTLE_TICKS;
    s_relay_err_ticks = 0u;
}

/** @brief 10ms 节拍推进: VPOWER 达标即定案, 超时才连续消抖判故障 (无 pending 时极速返回)
 */
void Gpio_RelayTick(void)
{
    uint16_t vpwr;

    if (s_relay_pending == 0u) return;

    /* 阶段1: 命中即定案 — VPOWER 达标说明触点已动作, 是物理事实, 不必再等时间 */
    vpwr = Vpower_GetVoltage();
    if (s_relay_target != 0u)
    {
        if (vpwr >= RELAY_ON_VPWR_MIN)                  /* ON: ≥20V = 触点已闭合 */
        {
            PSWErrRelay     = 0u;
            s_relay_pending = 0u;
            return;
        }
    }
    else
    {
        if (vpwr <= RELAY_OFF_VPWR_MAX)                 /* OFF: ≤2V = 触点已断开 */
        {
            PSWErrRelay     = 0u;
            s_relay_pending = 0u;
            return;
        }
    }

    /* 阶段2: 未达标先等 (预稳定窗口内 — 可能只是还没吸合完) */
    if (s_relay_ticks > 0u)
    {
        s_relay_ticks--;
        return;
    }

    /* 阶段3: 超时仍不达标 → 连续 3 拍异常才判故障 (防电源建立偏慢造成偶发误报) */
    if (s_relay_err_ticks < RELAY_DEBOUNCE_TICKS) { s_relay_err_ticks++; }
    if (s_relay_err_ticks >= RELAY_DEBOUNCE_TICKS)
    {
        PSWErrRelay     = 1u;
        s_relay_pending = 0u;
    }
}

/* ========================================================================== */
/*  8. UB 供电                                                                  */
/* ========================================================================== */

/** @brief UB 电压输出控制 (P13.6): 1=开启 0=关闭
 */
void Gpio_UBCtrl(uint8_t state)
{
    if (GPIO_IS_VALID(UB_GPIO_PORT, UB_GPIO_PIN))
    {
        Cy_GPIO_Write(UB_GPIO_PORT, UB_GPIO_PIN, (state != 0u) ? 1u : 0u);
    }
}

/* ========================================================================== */
/*  9. 指示灯                                                                   */
/* ========================================================================== */

/** @brief ABS 故障指示灯控制 (P18.0): 1=亮 0=灭
 */
void Gpio_ABSWLampCtrl(uint8_t state)
{
    if (GPIO_IS_VALID(ABS_WLAMP_GPIO_PORT, ABS_WLAMP_GPIO_PIN))
    {
        Cy_GPIO_Write(ABS_WLAMP_GPIO_PORT, ABS_WLAMP_GPIO_PIN, (state != 0u) ? 1u : 0u);
    }
}

/** @brief HSA 指示灯控制 (P18.4): 1=亮 0=灭
 */
void Gpio_HSALampCtrl(uint8_t state)
{
    if (GPIO_IS_VALID(HSA_LAMP_GPIO_PORT, HSA_LAMP_GPIO_PIN))
    {
        Cy_GPIO_Write(HSA_LAMP_GPIO_PORT, HSA_LAMP_GPIO_PIN, (state != 0u) ? 1u : 0u);
    }
}

/** @brief ASR 指示灯控制 (P11.2): 1=亮 0=灭
 */
void Gpio_ASRLampCtrl(uint8_t state)
{
    if (GPIO_IS_VALID(ASR_LAMP_GPIO_PORT, ASR_LAMP_GPIO_PIN))
    {
        Cy_GPIO_Write(ASR_LAMP_GPIO_PORT, ASR_LAMP_GPIO_PIN, (state != 0u) ? 1u : 0u);
    }
}

/* ========================================================================== */
/*  10. 接插件输入                                                              */
/* ========================================================================== */

/** @brief 接插件引脚状态读取 (低有效)
 *  @param xNum 接插件编号 (3 / 1)
 *  @param xPin 引脚编号 (4 / 6 / 5)
 *  @return 1 = 接地(低), 0 = 悬空(高), -1 = 参数非法
 */
int8_t Gpio_Xn_pin_StaGet(uint8_t xNum, uint8_t xPin)
{
    volatile stc_GPIO_PRT_t* port = NULL;
    uint32_t                 pin  = 0xFFu;

    if (xNum == 3u && xPin == 4u)           { port = X3_4_GPIO_PORT; pin = X3_4_GPIO_PIN; }
    else if (xNum == 1u && xPin == 6u)      { port = X1_6_GPIO_PORT; pin = X1_6_GPIO_PIN; }
    else if (xNum == 1u && xPin == 5u)      { port = X1_5_GPIO_PORT; pin = X1_5_GPIO_PIN; }
    else                                    { return -1; }

    if (!GPIO_IS_VALID(port, pin)) return 0;
    return (Cy_GPIO_Read(port, pin) == 0u) ? 1 : 0;
}

/* ========================================================================== */
/*  11. ASR 低边开关                                                             */
/*                                                                            */
/*  P2.0 = 前桥 (22口进 原FA_ASR + TR_ASR 共享), P6.2 = 后桥 (22口排 原DA_ASR 独享) */
/*  上电由 Bts724g_Init() 直写打开一次; 之后开/关全权由应用经 RTE 接口直写控制       */
/* ========================================================================== */

/** @brief 前桥 ASR 低边开关控制 (P2.0) — FarFA_in
 *  @param sw 非 0 = 闭合(拉低), 0 = 断开
 */
void Gpio_ASRFLowSideSw(uint32_t sw)
{
    if (GPIO_IS_VALID(ASR_F_LOWSIDE_CTRL_PORT, ASR_F_LOWSIDE_CTRL_PIN))
    {
        Cy_GPIO_Write(ASR_F_LOWSIDE_CTRL_PORT, ASR_F_LOWSIDE_CTRL_PIN,
                      (sw != 0u) ? 1u : 0u);
    }
}

/** @brief 前桥 ASR 低边开关状态回读 (硬件反馈)
 *  @return 1 = 闭合, 0 = 断开
 */
uint32_t Gpio_ASRFLowSideSt(void)
{
    if (!GPIO_IS_VALID(ASR_F_LOWSIDE_STAT_PORT, ASR_F_LOWSIDE_STAT_PIN)) return 0u;
    return (Cy_GPIO_Read(ASR_F_LOWSIDE_STAT_PORT, ASR_F_LOWSIDE_STAT_PIN) != 0u) ? 1u : 0u;
}

/** @brief 后桥 ASR 低边开关控制 (P6.2) — FarDA_in
 *  @param sw 非 0 = 闭合(拉低), 0 = 断开
 */
void Gpio_ASRRLowSideSw(uint8_t sw)
{
    if (GPIO_IS_VALID(ASR_R_LOWSIDE_CTRL_PORT, ASR_R_LOWSIDE_CTRL_PIN))
    {
        Cy_GPIO_Write(ASR_R_LOWSIDE_CTRL_PORT, ASR_R_LOWSIDE_CTRL_PIN,
                      (sw != 0u) ? 1u : 0u);
    }
}

/** @brief 后桥 ASR 低边开关状态回读 (硬件反馈)
 *  @return 1 = 闭合, 0 = 断开
 */
uint32_t Gpio_ASRRLowSideSt(void)
{
    if (!GPIO_IS_VALID(ASR_R_LOWSIDE_STAT_PORT, ASR_R_LOWSIDE_STAT_PIN)) return 0u;
    return (Cy_GPIO_Read(ASR_R_LOWSIDE_STAT_PORT, ASR_R_LOWSIDE_STAT_PIN) != 0u) ? 1u : 0u;
}
