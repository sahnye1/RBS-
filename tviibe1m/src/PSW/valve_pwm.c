/**
 * @file    valve_pwm.c
 * @brief   TCPWM 电磁阀 PWM 控制实现 (直写寄存器 + 强制 counter 溢出生效)
 *
 * @note    覆盖全部 17 个电磁阀, 统一用 TCPWM0 双比较模式
 *          时钟链: PERI_CLK(40MHz) → ÷400 → 100kHz (10μs/步)
 *          period 范围 1000~50000 → 10ms~500ms (常规控阀推荐);
 *                    硬上限 65535 步 = 655.35ms (16bit 计数),
 *                    VHT 保压标定测试用到 52000 步 (520ms)
 *          双比较模式: CC0→SET(脉冲起点), CC1→CLEAR(脉冲终点)
 *          SetDuty: 直写 CC0/CC1/PERIOD 后置 counter=period-1,
 *          → 下拍 overflow 触发, 新值从零点起效 (延迟 ≤ 1 拍 ≈ 10μs)
 *
 *          各阀引脚与 TCPWM LINE 分配: 见 bts724g.h 文件头映射表 / docs/阀映射表.md
 */

#include "valve_pwm.h"
#include "cy_project.h"
#include "cy_device_headers.h"
#include "peri_div.h"
#include "cmn.h"

/* ========================================================================== */
/*  宏定义                                                                      */
/* ========================================================================== */

/* ---- 共享时钟 ---- */
/* 目标 100kHz (10μs/步), 分频值由 periph_divider 动态计算 */
#define PWM_VALVE_TARGET_FREQ               100000u
#define PWM_VALVE_CLK_DIV_IDX               DIV16_NO_PWM_VALVE   /* 17 路共享分频器 #0 */

/* ---- 芯片0 (U6) 后桥 ---- */
#define PWM_RRI_TCPWM                       TCPWM0_GRP0_CNT32   /* P22.2 → LINE32 */
#define PWM_RRI_PCLK                        PCLK_TCPWM0_CLOCKS32
#define PWM_RRI_PORT                        GPIO_PRT22
#define PWM_RRI_PIN                         2u
#define PWM_RRI_MUX                         P22_2_TCPWM0_LINE32

#define PWM_RRO_TCPWM                       TCPWM0_GRP0_CNT33   /* P22.1 → LINE33 */
#define PWM_RRO_PCLK                        PCLK_TCPWM0_CLOCKS33
#define PWM_RRO_PORT                        GPIO_PRT22
#define PWM_RRO_PIN                         1u
#define PWM_RRO_MUX                         P22_1_TCPWM0_LINE33

#define PWM_LRO_TCPWM                       TCPWM0_GRP0_CNT34   /* P22.0 → LINE34 */
#define PWM_LRO_PCLK                        PCLK_TCPWM0_CLOCKS34
#define PWM_LRO_PORT                        GPIO_PRT22
#define PWM_LRO_PIN                         0u
#define PWM_LRO_MUX                         P22_0_TCPWM0_LINE34

#define PWM_LRI_TCPWM                       TCPWM0_GRP0_CNT37   /* P21.5 → LINE37 */
#define PWM_LRI_PCLK                        PCLK_TCPWM0_CLOCKS37
#define PWM_LRI_PORT                        GPIO_PRT21
#define PWM_LRI_PIN                         5u
#define PWM_LRI_MUX                         P21_5_TCPWM0_LINE37

/* ---- 芯片1 (U9) 前桥 ---- */
#define PWM_RFI_TCPWM                       TCPWM0_GRP0_CNT18   /* P0.0 → LINE18 */
#define PWM_RFI_PCLK                        PCLK_TCPWM0_CLOCKS18
#define PWM_RFI_PORT                        GPIO_PRT0
#define PWM_RFI_PIN                         0u
#define PWM_RFI_MUX                         P0_0_TCPWM0_LINE18

#define PWM_RFO_TCPWM                       TCPWM0_GRP0_CNT25   /* P23.4 → LINE25 */
#define PWM_RFO_PCLK                        PCLK_TCPWM0_CLOCKS25
#define PWM_RFO_PORT                        GPIO_PRT23
#define PWM_RFO_PIN                         4u
#define PWM_RFO_MUX                         P23_4_TCPWM0_LINE25

#define PWM_LFO_TCPWM                       TCPWM0_GRP1_CNT11   /* P23.3 → LINE267, GRP1 */
#define PWM_LFO_PCLK                        PCLK_TCPWM0_CLOCKS267
#define PWM_LFO_PORT                        GPIO_PRT23
#define PWM_LFO_PIN                         3u
#define PWM_LFO_MUX                         P23_3_TCPWM0_LINE267

#define PWM_LFI_TCPWM                       TCPWM0_GRP0_CNT31   /* P22.3 → LINE31 */
#define PWM_LFI_PCLK                        PCLK_TCPWM0_CLOCKS31
#define PWM_LFI_PORT                        GPIO_PRT22
#define PWM_LFI_PIN                         3u
#define PWM_LFI_MUX                         P22_3_TCPWM0_LINE31

/* ---- 芯片2 (U12) 21口 + TR_ASR ---- */
#define PWM_21IN_TCPWM                      TCPWM0_GRP0_CNT26   /* 21口进 P19.1 → LINE26 */
#define PWM_21IN_PCLK                       PCLK_TCPWM0_CLOCKS26
#define PWM_21IN_PORT                       GPIO_PRT19
#define PWM_21IN_PIN                        1u
#define PWM_21IN_MUX                        P19_1_TCPWM0_LINE26

#define PWM_21OUT_TCPWM                     TCPWM0_GRP1_CNT3    /* 21口排 P19.0 → LINE259, GRP1 */
#define PWM_21OUT_PCLK                      PCLK_TCPWM0_CLOCKS259
#define PWM_21OUT_PORT                      GPIO_PRT19
#define PWM_21OUT_PIN                       0u
#define PWM_21OUT_MUX                       P19_0_TCPWM0_LINE259

#define PWM_TR_ASR_TCPWM                    TCPWM0_GRP0_CNT51   /* TR_ASR P18.6 → LINE51 */
#define PWM_TR_ASR_PCLK                     PCLK_TCPWM0_CLOCKS51
#define PWM_TR_ASR_PORT                     GPIO_PRT18
#define PWM_TR_ASR_PIN                      6u
#define PWM_TR_ASR_MUX                      P18_6_TCPWM0_LINE51

/* ---- 芯片3 (U13) 22口 ---- */
#define PWM_22IN_TCPWM                      TCPWM0_GRP0_CNT54   /* 22口进 P18.3 → LINE54 */
#define PWM_22IN_PCLK                       PCLK_TCPWM0_CLOCKS54
#define PWM_22IN_PORT                       GPIO_PRT18
#define PWM_22IN_PIN                        3u
#define PWM_22IN_MUX                        P18_3_TCPWM0_LINE54

#define PWM_22OUT_TCPWM                     TCPWM0_GRP0_CNT52   /* 22口排 P18.5 → LINE52 */
#define PWM_22OUT_PCLK                      PCLK_TCPWM0_CLOCKS52
#define PWM_22OUT_PORT                      GPIO_PRT18
#define PWM_22OUT_PIN                       5u
#define PWM_22OUT_MUX                       P18_5_TCPWM0_LINE52

/* ---- 芯片4 (U19) 辅助桥 ---- */
#define PWM_RXI_TCPWM                       TCPWM0_GRP0_CNT41   /* P21.1 → LINE41 */
#define PWM_RXI_PCLK                        PCLK_TCPWM0_CLOCKS41
#define PWM_RXI_PORT                        GPIO_PRT21
#define PWM_RXI_PIN                         1u
#define PWM_RXI_MUX                         P21_1_TCPWM0_LINE41

#define PWM_RXO_TCPWM                       TCPWM0_GRP0_CNT42   /* P21.0 → LINE42 */
#define PWM_RXO_PCLK                        PCLK_TCPWM0_CLOCKS42
#define PWM_RXO_PORT                        GPIO_PRT21
#define PWM_RXO_PIN                         0u
#define PWM_RXO_MUX                         P21_0_TCPWM0_LINE42

#define PWM_LXO_TCPWM                       TCPWM0_GRP0_CNT28   /* P19.3 → LINE28 */
#define PWM_LXO_PCLK                        PCLK_TCPWM0_CLOCKS28
#define PWM_LXO_PORT                        GPIO_PRT19
#define PWM_LXO_PIN                         3u
#define PWM_LXO_MUX                         P19_3_TCPWM0_LINE28

#define PWM_LXI_TCPWM                       TCPWM0_GRP0_CNT27   /* P19.2 → LINE27 */
#define PWM_LXI_PCLK                        PCLK_TCPWM0_CLOCKS27
#define PWM_LXI_PORT                        GPIO_PRT19
#define PWM_LXI_PIN                         2u
#define PWM_LXI_MUX                         P19_2_TCPWM0_LINE27

/* ---- PWM 通用参数 ---- */
/* CC0=脉冲起点, CC1=脉冲终点                                              */
#define PWM_VALVE_PERIOD                    (1000u - 1u)  /* 默认 period_reg=999    */
/* "前段高"脉冲起点 = 0 (周期起点): counter 溢出置 SET、CC0 匹配置 CLEAR,
 * 故高电平从周期开始处立起 (CC0=0 且 CC1 永不匹配时即为恒高基态)。
 *
 * ⚠ 2026-09-15 修正 (**台架已确诊, 勿改回非 0**): 原为 5 (想避开 counter=0 竞争),
 *   结果是"恒高"驱动不成立 —— overflow 已在 counter=0 把输出拉低, 要等 CC0=5 才重新拉高,
 *   每个周期都在起点丢失一段高电平 → 实测排气阀占空只有 ~70% (6ms高/3ms低循环),
 *   排气阀(NO)保持不住、保压一路泄漏。改成 0 后 counter=0 处 CC0 的 SET 覆盖 overflow 的
 *   CLEAR → 真恒高、零毛刺, 台架恢复正常。
 *   (counter=0 处 SET 优先亦由进气阀 SetDuty(period, cc0_start=0, htime) 出脉冲验证) */
#define PWM_CC0_OFFSET                      0u            /* CC0 起点 = 周期起点 (前段高)      */
#define PWM_CC0_DISABLE                     0xFFFEu       /* CC0 很大 → 永不匹配, 恒 LOW       */
#define PWM_CC1_NEVER                       0xFFFEu       /* CC1 很大 → 永不匹配, 恒 HIGH      */

/* ========================================================================== */
/*  PWM 阀门查表                                                                 */
/* ========================================================================== */

typedef struct
{
    volatile stc_TCPWM_GRP_CNT_t* tcpwm;           /* TCPWM 计数器基址, NULL=GPIO 阀 */
} valve_pwm_map_t;

static const valve_pwm_map_t s_valve_pwm_map[VALVE_NUM_TOTAL] =
{
    /* 芯片0 (U6): 后桥 */
    [VALVE_RRI]    = { PWM_RRI_TCPWM    },
    [VALVE_RRO]    = { PWM_RRO_TCPWM    },
    [VALVE_LRO]    = { PWM_LRO_TCPWM    },
    [VALVE_LRI]    = { PWM_LRI_TCPWM    },
    /* 芯片1 (U9): 前桥 */
    [VALVE_RFI]    = { PWM_RFI_TCPWM    },
    [VALVE_RFO]    = { PWM_RFO_TCPWM    },
    [VALVE_LFO]    = { PWM_LFO_TCPWM    },
    [VALVE_LFI]    = { PWM_LFI_TCPWM    },
    /* 芯片2 (U12): 21口 + TR_ASR */
    [VALVE_21IN]   = { PWM_21IN_TCPWM   },
    [VALVE_21OUT]  = { PWM_21OUT_TCPWM  },
    [VALVE_TR_ASR] = { PWM_TR_ASR_TCPWM },
    /* 芯片3 (U13): 22口 */
    [VALVE_22IN]   = { PWM_22IN_TCPWM   },
    [VALVE_22OUT]  = { PWM_22OUT_TCPWM  },
    /* 芯片4 (U19): 辅助桥 */
    [VALVE_RXI]    = { PWM_RXI_TCPWM    },
    [VALVE_RXO]    = { PWM_RXO_TCPWM    },
    [VALVE_LXO]    = { PWM_LXO_TCPWM    },
    [VALVE_LXI]    = { PWM_LXI_TCPWM    },
};

/* ========================================================================== */
/*  PWM 配置 — 双比较模式 (直写 + counter 强制溢出生效)                           */
/*                                                                            */
/*  CC0 → SET (脉冲起点), CC1 → CLEAR (脉冲终点), overflow → CLEAR (基态 LOW)    */
/*                                                                            */
/*  counter: 0 ──→ CC0 ──→ CC1 ──→ period_reg(overflow)                        */
/*  output:  LOW    HIGH    LOW     LOW                                         */
/*           ↑       ↑       ↑       ↑                                          */
/*        overflow  CC0     CC1   overflow                                      */
/*         CLEAR    SET    CLEAR   CLEAR                                        */
/*                                                                            */
/*  SetDuty: 直写 CC0/CC1/PERIOD 后 set Counter=period-1 → 下拍溢出生效          */
/* ========================================================================== */

static const cy_stc_tcpwm_pwm_config_t s_pwm_cfg =
{
    .pwmMode            = CY_TCPWM_PWM_MODE_PWM,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .debug_pause        = false,
    .cc0MatchMode       = CY_TCPWM_PWM_TR_CTRL2_SET,       /* CC0 匹配 → 输出 HIGH (脉冲起点) */
    .cc1MatchMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR,     /* CC1 匹配 → 输出 LOW  (脉冲终点) */
    .overflowMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR,     /* 溢出 → 输出 LOW      (基态)    */
    .underflowMode      = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
    .deadTime           = 0ul, .deadTimeComp = 0ul,
    .runMode            = CY_TCPWM_PWM_CONTINUOUS,
    .period             = PWM_VALVE_PERIOD,
    .period_buff        = 0ul,  .enablePeriodSwap   = false,
    /* 上电初值: CC0/CC1 都设为"永不匹配" → 基态 CLEAR → 全部阀恒低(断电), 上电不会误开通
     * (不能用 PWM_CC0_OFFSET: 若为 0 会在 counter=0 触发 SET 而上电即开通) */
    .compare0           = PWM_CC0_DISABLE, .enableCompare0Swap  = false,
    .compare1           = PWM_CC1_NEVER,   .enableCompare1Swap  = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .invertPWMOut       = 0ul,  .invertPWMOutN       = 0ul,
    .killMode           = CY_TCPWM_PWM_STOP_ON_KILL,
    .switchInputMode    = CY_TCPWM_INPUT_LEVEL, .switchInput    = 0ul,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL, .reloadInput    = 0ul,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL, .startInput     = 0ul,
    .kill0InputMode     = CY_TCPWM_INPUT_LEVEL, .kill0Input     = 0ul,
    .kill1InputMode     = CY_TCPWM_INPUT_LEVEL, .kill1Input     = 0ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL, .countInput     = 1ul,
};

/* ========================================================================== */
/*  公共接口                                                                    */
/* ========================================================================== */

bool ValvePwm_IsPwmValve(valve_id_t valve_id)
{
    if (valve_id >= VALVE_NUM_TOTAL) return false;
    return (s_valve_pwm_map[valve_id].tcpwm != NULL);
}

/* ========================================================================== */
/*  初始化表 (由 ValvePwm_Init() 循环使用)                                      */
/* ========================================================================== */

typedef struct
{
    en_clk_dst_t               pclk;     /* PCLK 时钟目标 */
    volatile stc_GPIO_PRT_t*   port;     /* GPIO 端口 */
    uint32_t                   pin;      /* 引脚号 */
    en_hsiom_sel_t             hsiom;    /* HSIOM 复用选择 */
    volatile stc_TCPWM_GRP_CNT_t* tcpwm; /* TCPWM 计数器基址 */
} pwm_init_t;

static const pwm_init_t s_pwm_init[VALVE_NUM_TOTAL] =
{
    /* 芯片0 (U6): 后桥 */
    [VALVE_RRI]    = { PWM_RRI_PCLK,    PWM_RRI_PORT,    PWM_RRI_PIN,    PWM_RRI_MUX,    PWM_RRI_TCPWM    },
    [VALVE_RRO]    = { PWM_RRO_PCLK,    PWM_RRO_PORT,    PWM_RRO_PIN,    PWM_RRO_MUX,    PWM_RRO_TCPWM    },
    [VALVE_LRO]    = { PWM_LRO_PCLK,    PWM_LRO_PORT,    PWM_LRO_PIN,    PWM_LRO_MUX,    PWM_LRO_TCPWM    },
    [VALVE_LRI]    = { PWM_LRI_PCLK,    PWM_LRI_PORT,    PWM_LRI_PIN,    PWM_LRI_MUX,    PWM_LRI_TCPWM    },
    /* 芯片1 (U9): 前桥 */
    [VALVE_RFI]    = { PWM_RFI_PCLK,    PWM_RFI_PORT,    PWM_RFI_PIN,    PWM_RFI_MUX,    PWM_RFI_TCPWM    },
    [VALVE_RFO]    = { PWM_RFO_PCLK,    PWM_RFO_PORT,    PWM_RFO_PIN,    PWM_RFO_MUX,    PWM_RFO_TCPWM    },
    [VALVE_LFO]    = { PWM_LFO_PCLK,    PWM_LFO_PORT,    PWM_LFO_PIN,    PWM_LFO_MUX,    PWM_LFO_TCPWM    },
    [VALVE_LFI]    = { PWM_LFI_PCLK,    PWM_LFI_PORT,    PWM_LFI_PIN,    PWM_LFI_MUX,    PWM_LFI_TCPWM    },
    /* 芯片2 (U12): 21口 + TR_ASR */
    [VALVE_21IN]   = { PWM_21IN_PCLK,   PWM_21IN_PORT,   PWM_21IN_PIN,   PWM_21IN_MUX,   PWM_21IN_TCPWM   },
    [VALVE_21OUT]  = { PWM_21OUT_PCLK,  PWM_21OUT_PORT,  PWM_21OUT_PIN,  PWM_21OUT_MUX,  PWM_21OUT_TCPWM  },
    [VALVE_TR_ASR] = { PWM_TR_ASR_PCLK, PWM_TR_ASR_PORT, PWM_TR_ASR_PIN, PWM_TR_ASR_MUX, PWM_TR_ASR_TCPWM },
    /* 芯片3 (U13): 22口 */
    [VALVE_22IN]   = { PWM_22IN_PCLK,   PWM_22IN_PORT,   PWM_22IN_PIN,   PWM_22IN_MUX,   PWM_22IN_TCPWM   },
    [VALVE_22OUT]  = { PWM_22OUT_PCLK,  PWM_22OUT_PORT,  PWM_22OUT_PIN,  PWM_22OUT_MUX,  PWM_22OUT_TCPWM  },
    /* 芯片4 (U19): 辅助桥 */
    [VALVE_RXI]    = { PWM_RXI_PCLK,    PWM_RXI_PORT,    PWM_RXI_PIN,    PWM_RXI_MUX,    PWM_RXI_TCPWM    },
    [VALVE_RXO]    = { PWM_RXO_PCLK,    PWM_RXO_PORT,    PWM_RXO_PIN,    PWM_RXO_MUX,    PWM_RXO_TCPWM    },
    [VALVE_LXO]    = { PWM_LXO_PCLK,    PWM_LXO_PORT,    PWM_LXO_PIN,    PWM_LXO_MUX,    PWM_LXO_TCPWM    },
    [VALVE_LXI]    = { PWM_LXI_PCLK,    PWM_LXI_PORT,    PWM_LXI_PIN,    PWM_LXI_MUX,    PWM_LXI_TCPWM    },
};

/* ========================================================================== */
/*  初始化 (由 Bts724g_Init() 最后调用)                                         */
/* ========================================================================== */

void ValvePwm_Init(void)
{
    /* 共享 100kHz 时钟 (periph_divider 动态分频), 分配给全部 17 路 TCPWM */
    for (uint32_t i = 0u; i < VALVE_NUM_TOTAL; i++)
    {
        periph_divider(s_pwm_init[i].pclk, CY_SYSCLK_DIV_16_BIT, PWM_VALVE_CLK_DIV_IDX, PWM_VALVE_TARGET_FREQ);
    }

    /* GPIO 引脚配置 + TCPWM 初始化: 全部 17 阀统一用双比较模式 */
    for (uint32_t i = 0u; i < VALVE_NUM_TOTAL; i++)
    {
        const cy_stc_gpio_pin_config_t pwmPinCfg =
        {
            .outVal    = 0ul,
            .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
            .hsiom     = s_pwm_init[i].hsiom,
            .intEdge   = 0ul,  .intMask  = 0ul,
            .vtrip     = 0ul,  .slewRate = 0ul,
            .driveSel  = 0ul,
        };
        Cy_GPIO_Pin_Init(s_pwm_init[i].port, s_pwm_init[i].pin, &pwmPinCfg);
        Cy_Tcpwm_Pwm_Init(s_pwm_init[i].tcpwm, &s_pwm_cfg);
        Cy_Tcpwm_Pwm_Enable(s_pwm_init[i].tcpwm);
        Cy_Tcpwm_TriggerStart(s_pwm_init[i].tcpwm);
    }
}

/* ========================================================================== */
/*  PWM 参数幂等缓存                                                            */
/*                                                                            */
/*  上层 (ASW/COM) 周期性重复下发相同 period/htime。若每次都写寄存器, 无条件      */
/*  归零会反复腰斩当前周期 → 实际周期被软件调用间隔钉死 (排气阀 cc0=0 时尤其明显); */
/*  故在入口做"值未变化则直接 return"的短路, 避免重复重置周期。                   */
/*                                                                            */
/*  ⚠ 缓存失效: SetOnOff (开/关阀、检阀、诊断) 会直接改变输出, 必须让缓存失效,    */
/*     否则后续下发相同参数会被误短路 → 阀不动作。                               */
/* ========================================================================== */

static uint16_t s_duty_period[VALVE_NUM_TOTAL];
static uint16_t s_duty_cc0   [VALVE_NUM_TOTAL];
static uint16_t s_duty_htime [VALVE_NUM_TOTAL];
static bool     s_duty_valid [VALVE_NUM_TOTAL];

static bool ValvePwm_DutyIsSame(valve_id_t id, uint16_t period,
                                uint16_t cc0_start, uint16_t high_time)
{
    return (s_duty_valid[id] &&
            (s_duty_period[id] == period)   &&
            (s_duty_cc0[id]    == cc0_start) &&
            (s_duty_htime[id]  == high_time));
}

static void ValvePwm_DutyCacheSave(valve_id_t id, uint16_t period,
                                   uint16_t cc0_start, uint16_t high_time)
{
    s_duty_period[id] = period;
    s_duty_cc0[id]    = cc0_start;
    s_duty_htime[id]  = high_time;
    s_duty_valid[id]  = true;
}

/* S2 采样点 (test_deal 帧10/11): 写入接口实际收到的参数原值 (只存不计算) */
volatile uint16_t g_vpwmdbg_period[4] = {0u};
volatile uint16_t g_vpwmdbg_htime[4]  = {0u};
volatile uint8_t  g_vpwmdbg_count[4]  = {0u};

static void vpwm_dbg_rec(valve_id_t vid, uint16_t period, uint16_t high_time)
{
    uint8_t idx;
    if      (vid == VALVE_21IN)  { idx = 0u; }
    else if (vid == VALVE_21OUT) { idx = 1u; }
    else if (vid == VALVE_22IN)  { idx = 2u; }
    else if (vid == VALVE_22OUT) { idx = 3u; }
    else                         { return; }
    g_vpwmdbg_period[idx] = period;
    g_vpwmdbg_htime[idx]  = high_time;
    if (g_vpwmdbg_count[idx] < 0xFFu) { g_vpwmdbg_count[idx]++; }
}

/* ========================================================================== */
/*  控制接口                                                                    */
/*                                                                            */
/*  SetDuty(period, cc0_start, high_time):                                     */
/*    直写 PERIOD/CC1=(cc0_start+high_time)/CC0=cc0_start,                     */
/*    再强制 counter 跳到 period-1 → 下拍溢出生效, 零延迟                        */
/*                                                                            */
/*    进气阀 (后段高): cc0_start = period-htime, high_time = htime              */
/*    排气阀 (前段高): cc0_start = OFFSET(0, 周期起点), high_time = htime        */
/*                                                                            */
/*    高电平=0 → CC0=DISABLE 恒 LOW                                             */
/*    high_time≥period → CC1 ≥ PERIOD+1 → 恒 HIGH                              */
/*                                                                            */
/*  SetOnOff(true)  : CC0=OFFSET(0), CC1=NEVER → 恒 HIGH (真恒高, 无毛刺)         */
/*  SetOnOff(false) : CC0=DISABLE → 恒 LOW (直写,即时生效)                        */
/* ========================================================================== */

void ValvePwm_SetDuty(valve_id_t valve_id, uint16_t period,
                      uint16_t cc0_start, uint16_t high_time)
{
    vpwm_dbg_rec(valve_id, period, high_time);   /* S2 采样: 只存原值 */
    if (valve_id >= VALVE_NUM_TOTAL) return;

    volatile stc_TCPWM_GRP_CNT_t* const tcpwm = s_valve_pwm_map[valve_id].tcpwm;
    if (tcpwm == NULL) return;  /* GPIO 阀忽略 */

    if ((period == 0u) || (cc0_start >= period))
    {
        s_duty_valid[valve_id] = false;     /* 非法参数 → 缓存失效 (不短路后续合法值) */
        return;
    }

    /* 幂等短路: 参数与上次下发完全一致 → 不碰寄存器 */
    if (ValvePwm_DutyIsSame(valve_id, period, cc0_start, high_time))
    {
        return;
    }
    ValvePwm_DutyCacheSave(valve_id, period, cc0_start, high_time);

    if (high_time == 0u)
    {
        /* 0% 占空比: CC0 写 0xFFFE → 永不匹配 → 恒 LOW */
        Cy_Tcpwm_Pwm_SetCompare0(tcpwm, PWM_CC0_DISABLE);
        Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)(period - 1u));
        return;
    }

    const uint16_t period_reg = period - 1u;  /* 寄存器值 = 步数-1 */

    /* CC1 = cc0_start + high_time;
     * 若 high_time 使 CC1 超出 period → CC1 = period_reg+1 → 永不匹配 → 恒 HIGH */
    uint16_t cc1 = (uint16_t)(cc0_start + high_time);
    if (cc1 > period_reg)
    {
        cc1 = (uint16_t)(period_reg + 1u);  /* = period, 超出 period_reg 永不匹配 */
    }

    /* 写顺序: PERIOD → CC1(终点) → CC0(起点), 再强制 counter 跳到 period-1
     * 先设终点再设起点: 即使 CC0 立刻触发 SET, CC1 已就位。
     * 最后设 counter=period-1 → 下拍 overflow → CLEAR 基态 → counter 归零 →
     * CC0/CC1/PERIOD 新值从零点开始生效, 延迟 ≤ 1 拍 (10μs)。              */
    Cy_Tcpwm_Pwm_SetPeriod(tcpwm, (uint32_t)period_reg);
    Cy_Tcpwm_Pwm_SetCompare1(tcpwm, (uint32_t)cc1);
    Cy_Tcpwm_Pwm_SetCompare0(tcpwm, (uint32_t)cc0_start);
    Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)period_reg);
}

/* ========================================================================== */
/*  SetDutyNoReset — 动态调参 (智能衔接, 立即生效且无尾巴)                        */
/*                                                                            */
/*  【目标】 ABS 控阀动态调压: 改周期/占空比立即生效, 但波形不产生畸形"尾巴"        */
/*                                                                            */
/*  【与 SetDuty 的区别】                                                       */
/*    SetDuty       : 无条件 SetCounter(period-1) 归零 → 每次重开新周期           */
/*    SetDutyNoReset: 读当前 counter 智能判断, 仅在"会卡电平"时才归零              */
/*                                                                            */
/*  【智能判断逻辑 (双比较模式 CC0=SET 点)】                                      */
/*    写新 PERIOD/CC1/CC0 前先读 counter cur:                                   */
/*      ① cur >= 新CC0 且 cur <= 新period_reg → SET 点已错过会卡电平 → 归零      */
/*      ② cur <  新CC0                          → 还没到 SET 点, 自然衔接, 无尾巴 */
/*      ③ cur >  新period_reg                   → 已超新周期, TCPWM 自动溢出归零  */
/*                                                                            */
/*  【效果】                                                                     */
/*    500ms→100ms: cur>新周期 → 自动溢出立即切 (不等 500ms 跑完)                  */
/*    33%→75%    : cur 已过 SET → 归零, 避免尾巴                                 */
/*    75%→33%    : cur 未到 SET → 自然衔接, 波形连续                              */
/*                                                                            */
/*  ⚠ 已知缺陷 (2026-09-17 分析, 当前无调用方):                                  */
/*    不重置 counter 时, 若"新 CC0 尚未到达"而输出已处于高电平, 该高电平只能等     */
/*    CC1 匹配或 overflow 才被清除 → 新高电平时长失效                             */
/*    (例: 500ms/300ms 走到 200ms 时改为 500ms/100ms → 实际高电平 250ms, 非 100ms)。*/
/*    故本函数已停用 (代码保留供参考):                                           */
/*      · 无相位要求 (可接受脉冲截断) → 用 SetDuty                              */
/*      · "立即生效 + 相位连续"       → 用 SetDutyLive (写参数+归零 → 补 CC0 事件  */
/*        → 恢复 counter)                                                       */
/* ========================================================================== */
// void ValvePwm_SetDutyNoReset(valve_id_t valve_id, uint16_t period,
//                              uint16_t cc0_start, uint16_t high_time)
// {
//     if (valve_id >= VALVE_NUM_TOTAL) return;
//     volatile stc_TCPWM_GRP_CNT_t* const tcpwm = s_valve_pwm_map[valve_id].tcpwm;
//     if (tcpwm == NULL) return;  /* GPIO 阀忽略 */
//     if ((period == 0u) || (cc0_start >= period))
//     {
//         s_duty_valid[valve_id] = false;     /* 非法参数 → 缓存失效 */
//         return;
//     }
//     /* 幂等短路: 参数与上次下发完全一致 → 不碰寄存器 */
//     if (ValvePwm_DutyIsSame(valve_id, period, cc0_start, high_time))
//     {
//         return;
//     }
//     ValvePwm_DutyCacheSave(valve_id, period, cc0_start, high_time);
//     if (high_time == 0u)
//     {
//         /* 0% 占空比: 恒 LOW; 并强制溢出让"关阀"立即生效 ——
//          * 否则若当前正处于高电平段, 要等到原 CC1 才拉低 (最坏延迟近一个周期) */
//         Cy_Tcpwm_Pwm_SetCompare0(tcpwm, PWM_CC0_DISABLE);
//         Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)(period - 1u));
//         return;
//     }
//     const uint16_t period_reg = period - 1u;
//     uint16_t cc1 = (uint16_t)(cc0_start + high_time);
//     if (cc1 > period_reg)
//     {
//         cc1 = (uint16_t)(period_reg + 1u);  /* 超出周期永不匹配 → 恒 HIGH */
//     }
//     /* 写参数前先读当前 counter */
//     const uint32_t cur = Cy_Tcpwm_Pwm_GetCounter(tcpwm);
//     /* 写新 PERIOD/CC1/CC0 */
//     Cy_Tcpwm_Pwm_SetPeriod(tcpwm, (uint32_t)period_reg);
//     Cy_Tcpwm_Pwm_SetCompare1(tcpwm, (uint32_t)cc1);
//     Cy_Tcpwm_Pwm_SetCompare0(tcpwm, (uint32_t)cc0_start);
//     /* 智能衔接: 仅当 counter 已过 SET 点(CC0)且未超周期时归零, 避免尾巴
//      * 其余情况 (未到 CC0 自然衔接 / 已超周期自动溢出) 不干预 counter */
//     if (cur >= (uint32_t)cc0_start && cur <= (uint32_t)period_reg)
//     {
//         Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)period_reg);
//     }
// }

/* ========================================================================== */
/*  SetDutyLive — 动态调参: 参数立即生效 + 相位连续                              */
/*                                                                            */
/*                                                                            */
/*  【要解决的问题】                                                            */
/*    运行中改 period/htime 时:                                                 */
/*      - SetDuty       : 无条件归零 → 立即生效, 但当前脉冲被截断, 相位从头开始    */
/*      - SetDutyNoReset: 保相位但"新 CC0 未到 + 已是高电平"时高电平延续过久,      */
/*                        新 htime 失效                                        */
/*    本函数兼顾: 新参数立即生效, 且 counter 恢复到调用前位置 (相位连续)。          */
/*                                                                            */
/*  【实现 (① ② ③ 三步)】                                                       */
/*    ① 写新参数 + SetCounter(period-1) → 下拍 overflow, 让新周期从 0 建立基态    */
/*       (CC0=0 时 counter=0 处的 SET 会覆盖 overflow 的 CLEAR → 基态即高)        */
/*    ② 若原相位落在高电平段 (cc0 ≤ saved < cc1), 补触发一次 CC0(SET)            */
/*    ③ 恢复 saved → 相位连续; saved 超出新周期时硬件下拍自动溢出从头开始          */
/*                                                                            */
/*  【每步之间需要延时 ≥1 拍】: 时钟 100kHz → 1 拍 10μs, 取 11μs                 */
/*    总耗时 ≤ 4×11μs = 44μs, 仅参数变化时发生 (未变化由幂等短路跳过)。           */
/* ========================================================================== */

#define PWM_STEP_US     11u     /* ≥1 个 TCPWM 时钟拍 (10μs) */

void ValvePwm_SetDutyLive(valve_id_t valve_id, uint16_t period,
                          uint16_t cc0_start, uint16_t high_time)
{
    vpwm_dbg_rec(valve_id, period, high_time);   /* S2 采样: 只存原值 */
    if (valve_id >= VALVE_NUM_TOTAL) return;

    volatile stc_TCPWM_GRP_CNT_t* const tcpwm = s_valve_pwm_map[valve_id].tcpwm;
    if (tcpwm == NULL) return;  /* GPIO 阀忽略 */

    if ((period == 0u) || (cc0_start >= period))
    {
        s_duty_valid[valve_id] = false;     /* 非法参数 → 缓存失效 */
        return;
    }

    if (ValvePwm_DutyIsSame(valve_id, period, cc0_start, high_time))
    {
        return;                             /* 参数未变: 连相位也不该动 */
    }
    ValvePwm_DutyCacheSave(valve_id, period, cc0_start, high_time);

    const uint16_t period_reg = (uint16_t)(period - 1u);

    /* 0% 占空比 → 恒低, 无需恢复相位 */
    if (high_time == 0u)
    {
        Cy_Tcpwm_Pwm_SetCompare0(tcpwm, PWM_CC0_DISABLE);
        Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)period_reg);
        delay_us(PWM_STEP_US);
        return;
    }

    uint16_t cc1 = (uint16_t)(cc0_start + high_time);
    if (cc1 > period_reg)
    {
        cc1 = (uint16_t)(period_reg + 1u);  /* 超出周期 → 恒高, 永不匹配 CLEAR */
    }

    /* ① 记录原相位 + 写新参数 + 强制溢出建立基态 */
    const uint32_t saved = Cy_Tcpwm_Pwm_GetCounter(tcpwm);

    Cy_Tcpwm_Pwm_SetPeriod(tcpwm, (uint32_t)period_reg);
    Cy_Tcpwm_Pwm_SetCompare1(tcpwm, (uint32_t)cc1);
    Cy_Tcpwm_Pwm_SetCompare0(tcpwm, (uint32_t)cc0_start);
    Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)period_reg);
    delay_us(PWM_STEP_US);

    /* ② 原相位应处于高电平段 → 补一次 CC0(SET), 否则恢复 counter 后会缺这段高电平 */
    if ((saved >= (uint32_t)cc0_start) && (saved < (uint32_t)cc1))
    {
        if (cc0_start == 0u)
        {
            /* cc0=0: 靠"溢出 → counter=0 时 CC0 匹配"置高 */
            Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)period_reg);
        }
        else
        {
            Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)(cc0_start - 1u));
        }
        delay_us(PWM_STEP_US);
    }

    /* ③ 恢复原 counter → 相位连续 (超出新周期时下拍自动溢出, 等同从头开始) */
    /* ⚠ saved 可能超出新周期(周期变小): 若原样写回, counter 会超出周期范围,
     *   要数到 65535 才回绕(最长 655ms) → 期间不匹配 CC0/CC1 = 输出长时间不动,
     *   现象即"没有方波" (2026-10-09 排气阀 State=0x12 实测确诊)。仅在新范围内恢复。 */
    if (saved <= (uint32_t)period_reg)
    {
        Cy_Tcpwm_Pwm_SetCounter(tcpwm, saved);
    }
    delay_us(PWM_STEP_US);
}

void ValvePwm_SetOnOff(valve_id_t valve_id, bool on)
{
    if (valve_id >= VALVE_NUM_TOTAL) return;

    volatile stc_TCPWM_GRP_CNT_t* const tcpwm = s_valve_pwm_map[valve_id].tcpwm;
    if (tcpwm == NULL) return;  /* GPIO 阀忽略 */

    /* 开关操作直接改变输出 → PWM 参数缓存必须失效,
     * 否则随后下发相同 period/htime 会被幂等短路跳过 → 阀不动作 */
    s_duty_valid[valve_id] = false;

    if (on)
    {
        /* ⚠ 必须先恢复默认 PERIOD: 本函数用 CC0=PWM_CC0_OFFSET(0) 作为 SET 点、
         * CC1=NEVER 挡掉 CLEAR, 于是 counter=0 处 CC0 的 SET 覆盖 overflow 的 CLEAR
         * → 输出为**真正的恒高**。
         * ⚠ CC0 偏移不能用非 0 值 (旧值 5 会让每个周期起点丢一段高电平: 排气阀实测占空
         *   只有 ~70%, 6ms高/3ms低循环 → 阀保持不住、保压泄漏; 2026-09-15 台架确诊)。
         * 若该通道此前被 SetDuty 设过别的周期/占空, 也必须复位 PERIOD 才能保证恒高。 */
        Cy_Tcpwm_Pwm_SetPeriod(tcpwm, (uint32_t)PWM_VALVE_PERIOD);
        Cy_Tcpwm_Pwm_SetCompare0(tcpwm, PWM_CC0_OFFSET);
        Cy_Tcpwm_Pwm_SetCompare1(tcpwm, PWM_CC1_NEVER);
        Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)PWM_VALVE_PERIOD);   /* 强制溢出生效 */
    }
    else
    {
        Cy_Tcpwm_Pwm_SetCompare0(tcpwm, PWM_CC0_DISABLE);
        Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)PWM_VALVE_PERIOD);   /* 强制溢出生效 */
    }
}
