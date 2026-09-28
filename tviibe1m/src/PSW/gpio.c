/**
 * @file    gpio.c
 * @brief   GPIO 通用输出/输入控制 (继电器/UB/接插件/指示灯/ASR低边开关)
 *
 * @note    继电器 VPOWER 检测: Gpio_RelayCtrl() 动作后延迟 RELAY_TICK_DELAY 拍 (3 拍 = 30ms),
 *          由 Gpio_RelayTick() (每 10ms) 倒计时到期后读 VPOWER 判定开闭。
 */

#include "gpio.h"
#include "adc.h"
#include "psw_data.h"

/* ---- 地码输入 (P8.0, 低有效: 0=接地 1=悬空) ---- */
uint8_t Gpio_DiMaRead(void)
{
    if (!GPIO_IS_VALID(DIMA_GPIO_PORT, DIMA_GPIO_PIN)) return 1u;
    return (Cy_GPIO_Read(DIMA_GPIO_PORT, DIMA_GPIO_PIN) == 0u) ? 0u : 1u;
}

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
/*  继电器控制 + VPOWER 电压检测 (动作后 3 拍延迟, 非阻塞)                      */
/*                                                                            */
/*  ON  → 等 30ms (Gpio_RelayTick 推进) → VPOWER < 20V → PSWErrVpwrLo (开路)  */
/*  OFF → 等 30ms                        → VPOWER >  2V → PSWErrVpwrHi (闭合)  */
/* ========================================================================== */

#define RELAY_TICK_DELAY        3u      /* 3 拍 × 10ms = 30ms */
#define RELAY_ON_VPWR_MIN       200u    /* 20.0V */
#define RELAY_OFF_VPWR_MAX       20u    /*  2.0V */

static uint8_t s_relay_pending = 0u;
static uint8_t s_relay_target  = 0u;
static uint8_t s_relay_ticks   = 0u;

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

    s_relay_pending = 1u;
    s_relay_target  = (state != 0u) ? 1u : 0u;
    s_relay_ticks   = RELAY_TICK_DELAY;
}

/** @brief 10ms 节拍推进, 到期后读 VPOWER 判定继电器 (无 pending 时极速返回) */
void Gpio_RelayTick(void)
{
    if (s_relay_pending == 0u) return;
    if (--s_relay_ticks > 0u)  return;

    s_relay_pending = 0u;
    uint16_t vpwr = Vpower_GetVoltage();

    if (s_relay_target != 0u)
        PSWErrVpwrLo = (vpwr >= RELAY_ON_VPWR_MIN) ? 0u : 1u;
    else
        PSWErrVpwrHi = (vpwr <= RELAY_OFF_VPWR_MAX) ? 0u : 1u;
    PSWErrRelay = PSWErrVpwrLo | PSWErrVpwrHi;
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
/*  ASR 低边开关 (GPIO 输出 + 回读 + 引用计数)                                 */
/*                                                                            */
/*  P2.0 = 前桥 (22口进 原FA_ASR + TR_ASR 共享), P6.2 = 后桥 (22口排 原DA_ASR 独享) */
/*  引用计数仅在 0↔1 边界切换硬件, 防止重复操作                                  */
/* ========================================================================== */

static uint8_t s_asr_f_lowside_ref = 0u;
static uint8_t s_asr_r_lowside_ref = 0u;

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

void Gpio_ASRFLowSideEnable(uint8_t enable)
{
    if (enable != 0u)
    {
        if (s_asr_f_lowside_ref == 0u) { Gpio_ASRFLowSideSw(1u); }
        if (s_asr_f_lowside_ref < 255u) { s_asr_f_lowside_ref++; }
    }
    else
    {
        if (s_asr_f_lowside_ref > 0u)
        {
            s_asr_f_lowside_ref--;
            if (s_asr_f_lowside_ref == 0u) { Gpio_ASRFLowSideSw(0u); }
        }
    }
}

void Gpio_ASRRLowSideEnable(uint8_t enable)
{
    if (enable != 0u)
    {
        if (s_asr_r_lowside_ref == 0u) { Gpio_ASRRLowSideSw(1u); }
        if (s_asr_r_lowside_ref < 255u) { s_asr_r_lowside_ref++; }
    }
    else
    {
        if (s_asr_r_lowside_ref > 0u)
        {
            s_asr_r_lowside_ref--;
            if (s_asr_r_lowside_ref == 0u) { Gpio_ASRRLowSideSw(0u); }
        }
    }
}
