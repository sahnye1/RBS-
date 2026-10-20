/**
 * @file    gpio.c
 * @brief   GPIO 通用输出/输入控制 (继电器/UB/接插件/指示灯/ASR低边开关)
 *
 * @note    继电器 VPOWER 检测: Gpio_RelayCtrl() 动作后延迟 RELAY_TICK_DELAY 拍 (8 拍 = 80ms),
 *          由 Gpio_RelayTick() (每 10ms) 倒计时到期后读 VPOWER 判定开闭。
 */

#include "gpio.h"
#include "adc.h"
#include "psw_data.h"
#include "cmn.h"       /* g_skipHardWD / g_skipSoftWD (看门狗跳过标记) */

/* ========================================================================== */
/*  GPIO 引脚配置表                                                         */
/*                                                                            */
/*  字段: portReg, pinNum, outVal, driveMode, hsiom, intEdge, intMask,         */
/*        vtrip, slewRate, driveSel, vregEn, ibufMode, vtripSel, vrefSel, vohSel */
/* ========================================================================== */
static const cy_stc_gpio_pin_prt_config_t s_gpio_pin_cfg[] = {

    /* ---- 输出 ---- */
    {RELAY_GPIO_PORT, RELAY_GPIO_PIN, 1ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   /* 继电器 (P14.3), 上电闭合 */
    {UB_GPIO_PORT,    UB_GPIO_PIN,    1ul, CY_GPIO_DM_STRONG_IN_OFF, HSIOM_SEL_GPIO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   /* UB 供电 (P13.6), 上电开启 */

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
/*  GPIO 引脚初始化 (上电默认状态)                                               */
/*                                                                            */
/*  Cy_GPIO_Multi_Pin_Init 遍历数组, 内部逐条调用 Cy_GPIO_Pin_Init。            */
/* ========================================================================== */
void Gpio_Init(void)
{
    Cy_GPIO_Multi_Pin_Init(s_gpio_pin_cfg,
                           (uint32_t)(sizeof(s_gpio_pin_cfg) / sizeof(s_gpio_pin_cfg[0])));
}

/* ========================================================================== */
/*  双看门狗喂狗 — 2026-10-20 从 wd_feed.h(static inline) / cmn.c 统一到本文件维护 */
/*                                                                            */
/*  硬狗: P5.0 翻转, 经 AC 耦合驱动 QFAR2 (与 QFAR1 串联保持继电器闭合)          */
/*  软狗: 芯片内部 WDT 清零 (Upper Limit 由 SoftWD_init 设定)                   */
/*  跳过标记: g_skipHardWD / g_skipSoftWD = 1 时停喂 (PSWdebug 看门狗超时测试用) */
/* ========================================================================== */
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


/* ---- 地码输入 (P8.0, 低有效: 0=接地 1=悬空) ---- */
uint8_t Gpio_DiMaRead(void)
{
    if (!GPIO_IS_VALID(DIMA_GPIO_PORT, DIMA_GPIO_PIN)) return 1u;
    return (Cy_GPIO_Read(DIMA_GPIO_PORT, DIMA_GPIO_PIN) == 0u) ? 0u : 1u;
}

/* ========================================================================== */
/*  测试计时标记输出 — P22.0 / 电路 LROCN (后桥左排阀, 接插件 X2-5)               */
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
/*  继电器控制 + VPOWER 电压检测 (动作后 预稳定 + 连续消抖, 非阻塞)              */
/*                                                                            */
/*  ON  → 预稳定 80ms → 连续 3 拍 VPOWER ≥20V 判正常, 连续 3 拍 <20V 判开路     */
/*  OFF → 预稳定 80ms → 连续 3 拍 VPOWER ≤2V  判正常, 连续 3 拍 >2V  判闭合     */
/*  ⚠ 继电器故障(PSWErrRelay)与 VPOWER 电压报警(PSWErrVpwrLo/Hi, ADC 18V/36V)  */
/*    取值同源(都读 VPOWER)但为两条独立判断路, 本函数只写 PSWErrRelay。         */
/*  ⚠ 冷启动 VPOWER 建立时间不确定 → 预稳定后连续消抖, 不用单次采样判定。        */
/* ========================================================================== */

#define RELAY_SETTLE_TICKS      8u      /* 预稳定: 8 拍 × 10ms = 80ms (继电器吸合 + VPOWER 建立) */
#define RELAY_DEBOUNCE_TICKS    3u      /* 消抖: 连续 3 拍 (30ms) 正常/异常才定案 */
#define RELAY_ON_VPWR_MIN       200u    /* 20.0V */
#define RELAY_OFF_VPWR_MAX       20u    /*  2.0V */

static uint8_t s_relay_pending   = 0u;  /* 检测进行中 */
static uint8_t s_relay_target    = 0u;  /* 目标: 1=闭合 0=断开 */
static uint8_t s_relay_ticks     = 0u;  /* 预稳定倒计时 */
static uint8_t s_relay_ok_ticks  = 0u;  /* 连续正常计数 */
static uint8_t s_relay_err_ticks = 0u;  /* 连续异常计数 */

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
    s_relay_ok_ticks  = 0u;
    s_relay_err_ticks = 0u;
}

/** @brief 10ms 节拍推进: 预稳定窗口结束后连续消抖判定继电器 (无 pending 时极速返回) */
void Gpio_RelayTick(void)
{
    if (s_relay_pending == 0u) return;

    /* 阶段1: 预稳定窗口 (等继电器吸合 + VPOWER 建立) */
    if (s_relay_ticks > 0u)
    {
        s_relay_ticks--;
        return;
    }

    /* 阶段2: 连续消抖判定 (对称消抖, 连续 3 拍正常/异常才定案) */
    uint16_t vpwr = Vpower_GetVoltage();
    uint8_t  ok;
    if (s_relay_target != 0u)
    {
        ok = (vpwr >= RELAY_ON_VPWR_MIN) ? 1u : 0u;   /* ON: ≥20V 正常 */
    }
    else
    {
        ok = (vpwr <= RELAY_OFF_VPWR_MAX) ? 1u : 0u;  /* OFF: ≤2V 正常 */
    }

    if (ok != 0u)
    {
        s_relay_err_ticks = 0u;
        if (s_relay_ok_ticks < RELAY_DEBOUNCE_TICKS) { s_relay_ok_ticks++; }
        if (s_relay_ok_ticks >= RELAY_DEBOUNCE_TICKS)
        {
            PSWErrRelay = 0u;
            s_relay_pending = 0u;
        }
    }
    else
    {
        s_relay_ok_ticks = 0u;
        if (s_relay_err_ticks < RELAY_DEBOUNCE_TICKS) { s_relay_err_ticks++; }
        if (s_relay_err_ticks >= RELAY_DEBOUNCE_TICKS)
        {
            PSWErrRelay = 1u;
            s_relay_pending = 0u;
        }
    }
}

void Gpio_UBCtrl(uint8_t state)
{
    if (GPIO_IS_VALID(UB_GPIO_PORT, UB_GPIO_PIN))
    {
        Cy_GPIO_Write(UB_GPIO_PORT, UB_GPIO_PIN, (state != 0u) ? 1u : 0u);
    }
}

/* ========================================================================== */
/*  故障指示灯控制                                                              */
/* ========================================================================== */

void Gpio_ABSWLampCtrl(uint8_t state)
{
    if (GPIO_IS_VALID(ABS_WLAMP_GPIO_PORT, ABS_WLAMP_GPIO_PIN))
    {
        Cy_GPIO_Write(ABS_WLAMP_GPIO_PORT, ABS_WLAMP_GPIO_PIN, (state != 0u) ? 1u : 0u);
    }
}

void Gpio_HSALampCtrl(uint8_t state)
{
    if (GPIO_IS_VALID(HSA_LAMP_GPIO_PORT, HSA_LAMP_GPIO_PIN))
    {
        Cy_GPIO_Write(HSA_LAMP_GPIO_PORT, HSA_LAMP_GPIO_PIN, (state != 0u) ? 1u : 0u);
    }
}

void Gpio_ASRLampCtrl(uint8_t state)
{
    if (GPIO_IS_VALID(ASR_LAMP_GPIO_PORT, ASR_LAMP_GPIO_PIN))
    {
        Cy_GPIO_Write(ASR_LAMP_GPIO_PORT, ASR_LAMP_GPIO_PIN, (state != 0u) ? 1u : 0u);
    }
}

/* ========================================================================== */
/*  接插件引脚操作                                                             */
/* ========================================================================== */

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
/*  ASR 低边开关 (GPIO 输出 + 回读)                                 */
/*                                                                            */
/*  P2.0 = 前桥 (22口进 原FA_ASR + TR_ASR 共享), P6.2 = 后桥 (22口排 原DA_ASR 独享) */
/*  上电由 Bts724g_Init() 直写打开一次; 之后开/关全权由应用经 RTE 接口直写控制                                  */
/* ========================================================================== */


void Gpio_ASRFLowSideSw(uint32_t sw)
{
    if (GPIO_IS_VALID(ASR_F_LOWSIDE_CTRL_PORT, ASR_F_LOWSIDE_CTRL_PIN))
    {
        Cy_GPIO_Write(ASR_F_LOWSIDE_CTRL_PORT, ASR_F_LOWSIDE_CTRL_PIN,
                      (sw != 0u) ? 1u : 0u);
    }
}

uint32_t Gpio_ASRFLowSideSt(void)
{
    if (!GPIO_IS_VALID(ASR_F_LOWSIDE_STAT_PORT, ASR_F_LOWSIDE_STAT_PIN)) return 0u;
    return (Cy_GPIO_Read(ASR_F_LOWSIDE_STAT_PORT, ASR_F_LOWSIDE_STAT_PIN) != 0u) ? 1u : 0u;
}

void Gpio_ASRRLowSideSw(uint8_t sw)
{
    if (GPIO_IS_VALID(ASR_R_LOWSIDE_CTRL_PORT, ASR_R_LOWSIDE_CTRL_PIN))
    {
        Cy_GPIO_Write(ASR_R_LOWSIDE_CTRL_PORT, ASR_R_LOWSIDE_CTRL_PIN,
                      (sw != 0u) ? 1u : 0u);
    }
}

uint32_t Gpio_ASRRLowSideSt(void)
{
    if (!GPIO_IS_VALID(ASR_R_LOWSIDE_STAT_PORT, ASR_R_LOWSIDE_STAT_PIN)) return 0u;
    return (Cy_GPIO_Read(ASR_R_LOWSIDE_STAT_PORT, ASR_R_LOWSIDE_STAT_PIN) != 0u) ? 1u : 0u;
}

