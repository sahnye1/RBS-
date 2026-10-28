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
/*  1. 宏定义                                                                      */
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
#define PWM_CC0_OFFSET                      0u            /* CC0 起点 = 周期起点 (前段高)      */
#define PWM_CC0_DISABLE                     0xFFFEu       /* CC0 很大 → 永不匹配, 恒 LOW       */
#define PWM_CC1_NEVER                       0xFFFEu       /* CC1 很大 → 永不匹配, 恒 HIGH      */

/* ========================================================================== */
/*  2. PWM 阀门查表                                                                 */
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
/*  3. PWM 配置 (双比较模式)                           */
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
/*  4. 公共接口                                                                    */
/* ========================================================================== */

/** @brief 判断指定阀是否为 PWM 阀 (本工程 17 阀全部为 PWM 阀)
 *  @param valve_id 阀 ID / @return true=PWM 阀
 */
bool ValvePwm_IsPwmValve(valve_id_t valve_id)
{
    if (valve_id >= VALVE_NUM_TOTAL) return false;
    return (s_valve_pwm_map[valve_id].tcpwm != NULL);
}

/* ========================================================================== */
/*  5. 初始化表                                      */
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
/*  6. 初始化                                         */
/* ========================================================================== */

/** @brief PWM 阀通道初始化 (TCPWM 双比较模式 + 引脚复用 + 输出基态 LOW)
 *  @note  由 Bts724g_Init() 最后调用
 */
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
/*  7. PWM 参数幂等缓存                                                            */
/*                                                                            */
/*  上层 (ASW/COM) 周期性重复下发相同 period/htime。若每次都写寄存器, 无条件      */
/*  归零会反复腰斩当前周期 → 实际周期被软件调用间隔钉死 */
/*  故在入口做"值未变化则直接 return"的短路, 避免重复重置周期。                   */
/*                                                                            */
/*  ⚠ 缓存失效: SetOnOff (开/关阀、检阀、诊断) 会直接改变输出, 必须让缓存失效,    */
/*     否则后续下发相同参数会被误短路 → 阀不动作。                               */
/* ========================================================================== */

static uint16_t s_duty_period[VALVE_NUM_TOTAL];
static uint16_t s_duty_cc0   [VALVE_NUM_TOTAL];
static uint16_t s_duty_htime [VALVE_NUM_TOTAL];
static bool     s_duty_valid [VALVE_NUM_TOTAL];

/** @brief 判断下发的 period/cc0/htime 与缓存是否相同 (幂等短路用)
 *  @return true=参数未变, 可直接 return 避免重置周期
 */
static bool ValvePwm_DutyIsSame(valve_id_t id, uint16_t period,
                                uint16_t cc0_start, uint16_t high_time)
{
    return (s_duty_valid[id] &&
            (s_duty_period[id] == period)   &&
            (s_duty_cc0[id]    == cc0_start) &&
            (s_duty_htime[id]  == high_time));
}

/** @brief 保存本通道 PWM 参数到缓存 (供下次幂等判断)
 */
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

/* S2.5 采样点 (test_deal 帧12): SetDutyLive 执行路径
 *   0=未调用  1=参数非法return  2=幂等短路return  3=htime=0恒低
 *   4=高电平段立即生效(写寄存器)  5=低电平段(仅写参数, 下周期生效) */
volatile uint8_t  g_vpwmdbg_path[4]   = {0u};

/** @brief 阀 ID → PWM 调试采样索引
 *  @return 采样索引 (无效阀返回 0xFF)
 */
static uint8_t vpwm_dbg_idx(valve_id_t vid)
{
    if      (vid == VALVE_21IN)  { return 0u; }
    else if (vid == VALVE_21OUT) { return 1u; }
    else if (vid == VALVE_22IN)  { return 2u; }
    else if (vid == VALVE_22OUT) { return 3u; }
    else                         { return 0xFFu; }
}

/** @brief 记录 PWM 调试采样快照 (仅 VALVE_DIAG_DEBUG 调试用)
 *  @note  S3 采样点: 直读 TCPWM 寄存器, 不改变控制状态
 */
static void vpwm_dbg_rec(valve_id_t vid, uint16_t period, uint16_t high_time)
{
    const uint8_t idx = vpwm_dbg_idx(vid);
    if (idx >= 4u) { return; }
    g_vpwmdbg_period[idx] = period;
    g_vpwmdbg_htime[idx]  = high_time;
    if (g_vpwmdbg_count[idx] < 0xFFu) { g_vpwmdbg_count[idx]++; }
}

/* ========================================================================== */
/*  8. 控制接口                                                                    */
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

#define PWM_STEP_US     11u     /* ≥1 个 TCPWM 时钟拍 (10μs) */

/** @brief 动态调参 — 写新参数并归零建立基态 → 按需补一次 CC0(SET) → 恢复原 counter
 *  @note  新参数立即生效且不丢失当前相位 (不会截断当前脉冲); 参数未变由幂等短路挡掉
 */
void ValvePwm_SetDutyLive(valve_id_t valve_id, uint16_t period,
                          uint16_t cc0_start, uint16_t high_time)
{
    vpwm_dbg_rec(valve_id, period, high_time);   /* S2 采样: 只存原值 */
    if (valve_id >= VALVE_NUM_TOTAL) return;
    const uint8_t pidx = vpwm_dbg_idx(valve_id);

    volatile stc_TCPWM_GRP_CNT_t* const tcpwm = s_valve_pwm_map[valve_id].tcpwm;
    if (tcpwm == NULL) return;  /* GPIO 阀忽略 */

    if ((period == 0u) || (cc0_start >= period))
    {
        s_duty_valid[valve_id] = false;     /* 非法参数 → 缓存失效 */
        if (pidx < 4u) { g_vpwmdbg_path[pidx] = 1u; }   /* S2.5: 参数非法 */
        return;
    }

    if (ValvePwm_DutyIsSame(valve_id, period, cc0_start, high_time))
    {
        if (pidx < 4u) { g_vpwmdbg_path[pidx] = 2u; }   /* S2.5: 幂等短路 */
        return;                             /* 参数未变: 连相位也不该动 */
    }

    /* 记录"改前"参数 (用于判断当前输出是否在高电平段), 再覆盖缓存 */
    const bool     old_valid = s_duty_valid[valve_id];
    const uint16_t old_cc0   = s_duty_cc0[valve_id];
    const uint32_t old_cc1   = (uint32_t)s_duty_cc0[valve_id] + (uint32_t)s_duty_htime[valve_id];

    ValvePwm_DutyCacheSave(valve_id, period, cc0_start, high_time);

    const uint16_t period_reg = (uint16_t)(period - 1u);

    /* 0% 占空比 → 恒低, 无需恢复相位 */
    if (high_time == 0u)
    {
        Cy_Tcpwm_Pwm_SetCompare0(tcpwm, PWM_CC0_DISABLE);
        Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)period_reg);
        delay_us(PWM_STEP_US);
        if (pidx < 4u) { g_vpwmdbg_path[pidx] = 3u; }   /* S2.5: 恒低 */
        return;
    }

    uint16_t cc1 = (uint16_t)(cc0_start + high_time);
    if (cc1 > period_reg)
    {
        cc1 = (uint16_t)(period_reg + 1u);  /* 超出周期 → 恒高, 永不匹配 CLEAR */
    }

    /* ① 记录原相位 */
    const uint32_t saved = Cy_Tcpwm_Pwm_GetCounter(tcpwm);

    /* ★ 2026-10-19 改占空比分两种情况 (用户要求):
     *   - 改前【高电平已走完】(低电平段) → 本周期不干预, 新参数**下周期**生效
     *   - 改前【高电平未走完】(高电平段) → **立即生效**
     *       · 新高电平更短 → 立即截断 (变低)
     *       · 新高电平更长 → 延续到新终点
     *   ⚠ 缓存无效(首次/SetOnOff 之后)无法判断旧电平 → 按"高电平段"处理(立即生效, 保持原行为) */
    const bool was_high = (old_valid == false) ||
                          ((saved >= (uint32_t)old_cc0) && (saved < old_cc1));

    if (was_high == false)
    {
        /* 情况1: 低电平段 → 只写新参数, 不动 counter (本周期高电平已过, 无需处理) */
        Cy_Tcpwm_Pwm_SetPeriod(tcpwm, (uint32_t)period_reg);
        Cy_Tcpwm_Pwm_SetCompare1(tcpwm, (uint32_t)cc1);
        Cy_Tcpwm_Pwm_SetCompare0(tcpwm, (uint32_t)cc0_start);

        /* 仅当周期变小导致 counter 超出新周期时才强制溢出
         * (否则 counter 会一路数到 65535 才回绕 = 长死区, 2026-10-09 排气阀实测踩过) */
        if (saved > (uint32_t)period_reg)
        {
            Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)period_reg);
            delay_us(PWM_STEP_US);
        }

        if (pidx < 4u) { g_vpwmdbg_path[pidx] = 5u; }   /* S2.5: 低电平段, 下周期生效 */
        return;
    }

    /* 情况2: 高电平段 → 写新参数 + 强制溢出建立基态 (旧周期在此终止) */
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

    if (pidx < 4u) { g_vpwmdbg_path[pidx] = 4u; }   /* S2.5: 正常写寄存器 */
}

/** @brief 阀开关控制 — true=恒高 (真恒高无毛刺) / false=恒低 (直写即时生效)
 *  @note  会直接改变输出, 故必须让 PWM 参数缓存失效
 */
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
        /* 先恢复默认 PERIOD, 否则 counter 从 999 递增到当前大周期(如 500ms)才溢出,
         * 期间 CC0=0xFFFE 不 SET、输出保持高 → 关阀延迟近一个周期 (~490ms)。
         * 恢复 PERIOD=999 后 SetCounter(999) 下拍即溢出 → 立即 CLEAR 变低。 */
        Cy_Tcpwm_Pwm_SetPeriod(tcpwm, (uint32_t)PWM_VALVE_PERIOD);
        Cy_Tcpwm_Pwm_SetCompare0(tcpwm, PWM_CC0_DISABLE);
        Cy_Tcpwm_Pwm_SetCounter(tcpwm, (uint32_t)PWM_VALVE_PERIOD);   /* 强制溢出生效 */
    }
}

/* ========================================================================== */
/*  9. S3 采样点 (调试)                      */
/* ========================================================================== */

/** @brief 读取 PWM 寄存器快照 (仅调试, 不改变控制状态)
 */
void ValvePwm_GetRegs(valve_id_t valve_id, uint16_t *period,
                      uint16_t *cc0, uint16_t *cc1, uint16_t *counter)
{
    *period = 0xFFFFu; *cc0 = 0xFFFFu; *cc1 = 0xFFFFu; *counter = 0xFFFFu;

    if (valve_id >= VALVE_NUM_TOTAL) return;

    volatile stc_TCPWM_GRP_CNT_t* const tcpwm = s_valve_pwm_map[valve_id].tcpwm;
    if (tcpwm == NULL) return;   /* GPIO 阀 */

    *period  = (uint16_t)Cy_Tcpwm_Pwm_GetPeriod(tcpwm);
    *cc0     = (uint16_t)Cy_Tcpwm_Pwm_GetCompare0(tcpwm);
    *cc1     = (uint16_t)Cy_Tcpwm_Pwm_GetCompare1(tcpwm);
    *counter = (uint16_t)Cy_Tcpwm_Pwm_GetCounter(tcpwm);
}
