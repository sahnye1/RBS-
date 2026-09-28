/**
 * @file    bts724g.c
 * @brief   BTS724G 五片阀驱动器 — IN 控制 + ST 回读 + 开路/短路诊断
 *
 * @details 诊断架构 (阀映射见 docs/阀映射表.md):
 *          ValveDiag_Process() 每 10ms 调用一次, 40ms (s_call_count & 3) 执行 1 步,
 *          4 步轮转:
 *            OPEN   步: 阀开路 — 开 ST 伙伴 → 读目标阀 OFF 态 ST → Low = 开路
 *                       (ST 开漏低有效); 独占 ST 的阀 (TR_ASR): 开自身 → 读 ON 态 ST
 *            SHORT  步: 阀短路 — 同组全关后仅开目标阀 → 读 FAR → High = 对地短路
 *            CHIP_O 步: 芯片开路 — 全 IN=HIGH → 两路 ST 均 Low = 芯片驱动级故障
 *            CHIP_S 步: 芯片短路 — 全 IN=LOW  → FAR = Low  = 芯片内部短路
 *          阀级 3 次、芯片级 5 次上下计数消抖后写入 g_bts724g_fault_status。
 *          全覆盖: 17 阀 × 2 步 + 5 芯片 × 2 步 = 44 步 × 40ms ≈ 1.76s。
 */

#include "bts724g.h"
#include "valve_pwm.h"
#include "psw_data.h"
#include "cy_project.h"
#include "cy_device_headers.h"
#include "RTE.h"

/* ========================================================================== */
/*  常量与宏                                                                   */
/* ========================================================================== */

/* 故障确认: 候选值必须连续 N 次相同才提交 (防单次采样噪声) */
#define FAULT_CONFIRM_NEED        3u    /* 阀级故障确认次数 */

/* 芯片级故障确认次数 (高于阀级, 降低阀故障干扰导致的芯片误报) */
#define CHIP_FAULT_CONFIRM_NEED   5u

/* GPIO 轮询参数 */
#define VALVE_ON_SETTLE_US  320u   /* BTS724G 输出通道开通建立时间 */
#define LOWSIDE_SETTLE_US   300u   /* 低边开关开通→芯片上电稳定时间 */
#define CHIP_SETTLE_US        3u   /* 芯片检测 IN 切换稳定时间 */

/* ========================================================================== */
/*  硬件配置表                                                                 */
/* ========================================================================== */

/* ---- 芯片 GPIO 引脚配置 ---- */
typedef struct
{
    volatile stc_GPIO_PRT_t* in_port[BTS724G_NUM_CH_PER_CHIP];
    uint32_t                 in_pin[BTS724G_NUM_CH_PER_CHIP];
    volatile stc_GPIO_PRT_t* st12_port;      /* ST1/2 线与端口 */
    uint32_t                 st12_pin;
    volatile stc_GPIO_PRT_t* st34_port;      /* ST3/4 线与端口 */
    uint32_t                 st34_pin;
    uint8_t                  num_valves;     /* 实际阀数 (1~4) */
} bts724g_chip_config_t;

static const bts724g_chip_config_t g_chip_cfg[BTS724G_NUM_CHIPS] =
{
    /* U6 后桥: IN1=P22.2 RRI / IN2=P22.1 RRO / IN3=P22.0 LRO / IN4=P21.5 LRI
     *          ST12=P5.2 (RRI/RRO), ST34=P5.1 (LRO/LRI), 短路组 A (FAR_CK2 P11.1) */
    [0] = {
        .in_port   = {GPIO_PRT22, GPIO_PRT22, GPIO_PRT22, GPIO_PRT21},
        .in_pin    = {2u,         1u,         0u,         5u         },
        .st12_port = GPIO_PRT5,  .st12_pin = 2u,
        .st34_port = GPIO_PRT5,  .st34_pin = 1u,
        .num_valves = 4u,
    },
    /* U9 前桥: IN1=P0.0 RFI / IN2=P23.4 RFO / IN3=P23.3 LFO / IN4=P22.3 LFI
     *          ST12=P6.0 (RFI/RFO), ST34=P5.3 (LFO/LFI), 短路组 A (FAR_CK2 P11.1) */
    [1] = {
        .in_port   = {GPIO_PRT0,  GPIO_PRT23, GPIO_PRT23, GPIO_PRT22},
        .in_pin    = {0u,         4u,         3u,         3u         },
        .st12_port = GPIO_PRT6,  .st12_pin = 0u,
        .st34_port = GPIO_PRT5,  .st34_pin = 3u,
        .num_valves = 4u,
    },
    /* U12 21口: IN1=P19.1 21口进 / IN2=P19.0 21口排 / IN3=P18.6 TR_ASR
     *           (IN3/IN4 板级并接 P18.6, OUT3/OUT4 并联驱动同一线圈 → 一路 PWM 驱动双通道,
     *            故 IN4 不需另配引脚; 详见 bts724g.h 头部说明)
     *           ST12=P2.3 (21口进/排), ST34=P2.2 (TR_ASR 独占), 短路组 B (FAR_CK P17.2) */
    [2] = {
        .in_port   = {GPIO_PRT19, GPIO_PRT19, GPIO_PRT18, NULL      },
        .in_pin    = {1u,         0u,         6u,         0u         },
        .st12_port = GPIO_PRT2,  .st12_pin = 3u,
        .st34_port = GPIO_PRT2,  .st34_pin = 2u,
        .num_valves = 3u,
    },
    /* U13 22口: IN1=P18.3 22口进 / IN2=P18.5 22口排 (IN3/IN4 未用, 无 ST34)
     *           ST12=P0.1, 短路组 B (FAR_CK P17.2) */
    [3] = {
        .in_port   = {GPIO_PRT18, GPIO_PRT18, NULL,      NULL      },
        .in_pin    = {3u,         5u,         0u,         0u         },
        .st12_port = GPIO_PRT0,  .st12_pin = 1u,
        .st34_port = NULL,       .st34_pin = 0u,
        .num_valves = 2u,
    },
    /* U19 辅助桥: IN1=P21.1 RXI / IN2=P21.0 RXO / IN3=P19.3 LXO / IN4=P19.2 LXI
     *             ST12=P14.2 (RXI/RXO), ST34=P3.1 (LXO/LXI), 短路组 B (FAR_CK P17.2) */
    [4] = {
        .in_port   = {GPIO_PRT21, GPIO_PRT21, GPIO_PRT19, GPIO_PRT19},
        .in_pin    = {1u,         0u,         3u,         2u         },
        .st12_port = GPIO_PRT14, .st12_pin = 2u,
        .st34_port = GPIO_PRT3,  .st34_pin = 1u,
        .num_valves = 4u,
    },
};

/* ---- 阀 → 芯片+通道 映射 ---- */
typedef struct
{
    uint8_t      chip_id;
    uint8_t      ch_id;         /* 0=IN1, 1=IN2, 2=IN3, 3=IN4 */
    const char*  name;
} bts724g_valve_map_t;

static const bts724g_valve_map_t g_bts724g_valve_map[VALVE_NUM_TOTAL] =
{
    [VALVE_RRI]    = {0u, 0u, "RRI"},
    [VALVE_RRO]    = {0u, 1u, "RRO"},
    [VALVE_LRO]    = {0u, 2u, "LRO"},
    [VALVE_LRI]    = {0u, 3u, "LRI"},
    [VALVE_RFI]    = {1u, 0u, "RFI"},
    [VALVE_RFO]    = {1u, 1u, "RFO"},
    [VALVE_LFO]    = {1u, 2u, "LFO"},
    [VALVE_LFI]    = {1u, 3u, "LFI"},
    [VALVE_21IN]   = {2u, 0u, "21IN"},     /* 21口 进气阀 (原 TRFIN) */
    [VALVE_21OUT]  = {2u, 1u, "21OUT"},    /* 21口 排气阀 (原 TRFOUT) */
    [VALVE_TR_ASR] = {2u, 2u, "TR_ASR"},   /* TR_ASR 阀 */
    [VALVE_22IN]   = {3u, 0u, "22IN"},     /* 22口 进气阀 (原 FA_ASR) */
    [VALVE_22OUT]  = {3u, 1u, "22OUT"},    /* 22口 排气阀 (原 DA_ASR) */
    [VALVE_RXI]    = {4u, 0u, "RXI"},
    [VALVE_RXO]    = {4u, 1u, "RXO"},
    [VALVE_LXO]    = {4u, 2u, "LXO"},
    [VALVE_LXI]    = {4u, 3u, "LXI"},
};

/* ---- ST 共享伙伴表 (与 docs/阀映射表.md "ST伙伴" 列一致) ----
 * BTS724G 每 2 通道共享 1 个 ST 组 (开漏线与): IN1/IN2→ST12, IN3/IN4→ST34。
 * 测试某阀时 partner 必须关闭, 否则其 ST 状态会污染测试结果。
 * 无伙伴 (独占 ST) 时值为 VALVE_NUM_TOTAL — 仅 TR_ASR (U12 IN3, 独占 P2.2)。 */
static const valve_id_t s_st_partner[VALVE_NUM_TOTAL] =
{
    [VALVE_RRI]    = VALVE_RRO,    [VALVE_RRO]    = VALVE_RRI,
    [VALVE_LRO]    = VALVE_LRI,    [VALVE_LRI]    = VALVE_LRO,
    [VALVE_RFI]    = VALVE_RFO,    [VALVE_RFO]    = VALVE_RFI,
    [VALVE_LFO]    = VALVE_LFI,    [VALVE_LFI]    = VALVE_LFO,
    [VALVE_21IN]   = VALVE_21OUT,  [VALVE_21OUT]  = VALVE_21IN,
    [VALVE_TR_ASR] = (valve_id_t)VALVE_NUM_TOTAL,  /* 独占 ST34, 无伙伴 */
    [VALVE_22IN]   = VALVE_22OUT,  [VALVE_22OUT]  = VALVE_22IN,
    [VALVE_RXI]    = VALVE_RXO,    [VALVE_RXO]    = VALVE_RXI,
    [VALVE_LXO]    = VALVE_LXI,    [VALVE_LXI]    = VALVE_LXO,
};

/* ---- 阀 → 短路检测组 映射 ---- */
static const short_group_id_t s_bts724g_valve_to_short_group[VALVE_NUM_TOTAL] =
{
    [VALVE_RRI]    = SHORT_GROUP_A, [VALVE_RRO]    = SHORT_GROUP_A,
    [VALVE_LRO]    = SHORT_GROUP_A, [VALVE_LRI]    = SHORT_GROUP_A,
    [VALVE_RFI]    = SHORT_GROUP_A, [VALVE_RFO]    = SHORT_GROUP_A,
    [VALVE_LFO]    = SHORT_GROUP_A, [VALVE_LFI]    = SHORT_GROUP_A,
    [VALVE_21IN]   = SHORT_GROUP_B, [VALVE_21OUT]  = SHORT_GROUP_B,
    [VALVE_TR_ASR] = SHORT_GROUP_B,
    [VALVE_22IN]   = SHORT_GROUP_B, [VALVE_22OUT]  = SHORT_GROUP_B,
    [VALVE_RXI]    = SHORT_GROUP_B, [VALVE_RXO]    = SHORT_GROUP_B,
    [VALVE_LXO]    = SHORT_GROUP_B, [VALVE_LXI]    = SHORT_GROUP_B,
};

/* ---- 芯片 → 短路检测组 映射 ---- */
static const short_group_id_t s_chip_short_group[BTS724G_NUM_CHIPS] =
{
    SHORT_GROUP_A,  /* U6  — FAR_CK2 (P11.1) */
    SHORT_GROUP_A,  /* U9  — FAR_CK2 (P11.1) */
    SHORT_GROUP_B,  /* U12 — FAR_CK  (P17.2) */
    SHORT_GROUP_B,  /* U13 — FAR_CK  (P17.2) */
    SHORT_GROUP_B,  /* U19 — FAR_CK  (P17.2) */
};

/* ========================================================================== */
/*  全局状态                                                                   */
/* ========================================================================== */

volatile bts724g_fault_status_t g_bts724g_fault_status = {0};
volatile uint32_t               g_systick_ms = 0u;

/* 应用层上报的"阀使用中"状态 (ValveDiag_Process 跳过工作中的阀) */
static volatile bool s_valve_active[VALVE_NUM_TOTAL] = { false };

/* 候选值累积器 (3 次上下计数消抖) */
static uint8_t s_open_candidate_count[VALVE_NUM_TOTAL]  = {0};
static uint8_t s_short_candidate_count[VALVE_NUM_TOTAL] = {0};

/* 诊断步进计数器 */
static uint32_t s_call_count = 0u;

/* ========================================================================== */
/*  内部辅助函数                                                               */
/* ========================================================================== */

/** @brief 读取芯片的 ST 组状态。 @return true=High(正常), false=Low(故障) */
static bool ReadST(uint8_t chip, uint8_t st_group)
{
    if (chip >= BTS724G_NUM_CHIPS) return false;
    const bts724g_chip_config_t* cfg = &g_chip_cfg[chip];
    volatile stc_GPIO_PRT_t* port;
    uint32_t pin;

    if (st_group == 0u) { port = cfg->st12_port; pin = cfg->st12_pin; }
    else                { port = cfg->st34_port; pin = cfg->st34_pin; }

    if (port == NULL) return false;
    return (Cy_GPIO_Read(port, pin) != 0u);
}

/* 前向声明: ASR 低边开关引用计数接口 (定义在 rte_psw.c) */
extern void ASRFLowSideEnable(uint8_t enable);
extern void ASRRLowSideEnable(uint8_t enable);

/** @brief ASR 低边开关映射 — 22口进(原FA_ASR)/TR_ASR/21口/辅助桥阀共享 P2.0 (前 ASR),
 *         22口排(原DA_ASR) 用 P6.2 (后 ASR) */
static void bts724g_asr_lowside_enable(valve_id_t vid, uint8_t enable)
{
    switch (vid)
    {
    case VALVE_22IN:
    case VALVE_TR_ASR:
    case VALVE_21IN:
    case VALVE_21OUT:
    case VALVE_RXI://后续注释
    case VALVE_RXO://后续注释
    case VALVE_LXO://后续注释
    case VALVE_LXI://后续注释
        ASRFLowSideEnable(enable);
        break;
    case VALVE_22OUT:
        ASRRLowSideEnable(enable);
        break;
    default:
        break;
    }
}

/* ========================================================================== */
/*  FAR 短路检测电路 (GPIO 读取)                                               */
/*                                                                            */
/*  A 组 (U6+U9, 8阀)     → 9013 → P11.1 (TTL 阈值)                            */
/*  B 组 (U19+U12+U13, 9阀) → 9013 → P17.2 (TTL 阈值)                          */
/*                                                                            */
/*  正常: OUT=24V → 9013 导通 → FAR≈0.27V → 读 Low                             */
/*  短路: OUT≈0V  → 9013 截止 → FAR≈3.0V  → 读 High                            */
/* ========================================================================== */

void ValveShort_Init(void)
{
    const cy_stc_gpio_pin_config_t farPinCfg =
    {
        .outVal = 0ul, .driveMode = CY_GPIO_DM_HIGHZ,
        .hsiom = P11_1_GPIO, .intEdge = 0ul, .intMask = 0ul,
        .vtrip = CY_GPIO_VTRIP_TTL,
        .slewRate = 0ul, .driveSel = 0ul,
    };
    Cy_GPIO_Pin_Init(FAR_GPIO_PORT_A, FAR_GPIO_PIN_A, &farPinCfg);

    cy_stc_gpio_pin_config_t farPinCfgB = farPinCfg;
    farPinCfgB.hsiom = P17_2_GPIO;
    Cy_GPIO_Pin_Init(FAR_GPIO_PORT_B, FAR_GPIO_PIN_B, &farPinCfgB);
}

bool ValveShort_ReadGroup(short_group_id_t group)
{
    if (group >= SHORT_GROUP_NUM) return false;
    if (group == SHORT_GROUP_A)
    {
        return (Cy_GPIO_Read(FAR_GPIO_PORT_A, FAR_GPIO_PIN_A) != 0u);
    }
    else
    {
        return (Cy_GPIO_Read(FAR_GPIO_PORT_B, FAR_GPIO_PIN_B) != 0u);
    }
}

/* ========================================================================== */
/*  公开 API — 初始化与阀控制                                                  */
/* ========================================================================== */

/**
 * @brief 初始化所有 BTS724G GPIO 引脚。
 *        INx: 推挽输出 Low (阀关闭)  STx: HIGHZ 输入 (外部 5.1k 上拉)
 */
void Bts724g_Init(void)
{
    for (uint8_t chip = 0u; chip < BTS724G_NUM_CHIPS; chip++)
    {
        const bts724g_chip_config_t* cfg = &g_chip_cfg[chip];

        /* INx: 推挽输出 Low */
        for (uint8_t ch = 0u; ch < cfg->num_valves; ch++)
        {
            Cy_GPIO_Write(cfg->in_port[ch], cfg->in_pin[ch], 0u);
            Cy_GPIO_SetHSIOM(cfg->in_port[ch], cfg->in_pin[ch], HSIOM_SEL_GPIO);
            Cy_GPIO_SetDrivemode(cfg->in_port[ch], cfg->in_pin[ch], CY_GPIO_DM_STRONG);
        }

        /* 未用通道 INx 也初始化为推挽输出 Low (防浮空) */
        for (uint8_t ch = cfg->num_valves; ch < BTS724G_NUM_CH_PER_CHIP; ch++)
        {
            if (cfg->in_port[ch] != NULL)
            {
                Cy_GPIO_Write(cfg->in_port[ch], cfg->in_pin[ch], 0u);
                Cy_GPIO_SetHSIOM(cfg->in_port[ch], cfg->in_pin[ch], HSIOM_SEL_GPIO);
                Cy_GPIO_SetDrivemode(cfg->in_port[ch], cfg->in_pin[ch], CY_GPIO_DM_STRONG);
            }
        }

        /* ST: HIGHZ 输入 */
        if (cfg->st12_port != NULL)
        {
            Cy_GPIO_SetHSIOM(cfg->st12_port, cfg->st12_pin, HSIOM_SEL_GPIO);
            Cy_GPIO_SetDrivemode(cfg->st12_port, cfg->st12_pin, CY_GPIO_DM_HIGHZ);
        }
        if (cfg->st34_port != NULL)
        {
            Cy_GPIO_SetHSIOM(cfg->st34_port, cfg->st34_pin, HSIOM_SEL_GPIO);
            Cy_GPIO_SetDrivemode(cfg->st34_port, cfg->st34_pin, CY_GPIO_DM_HIGHZ);
        }
    }

    ValveShort_Init();
    ValvePwm_Init();
}

/**
 * @brief 设置阀的开关状态 (PWM 阀委托 ValvePwm)。
 */
void Bts724g_SetValve(valve_id_t valve_id, bool on)
{
    if (valve_id >= VALVE_NUM_TOTAL) return;

    /* 所有 17 阀均为 PWM 阀 (IN 脚为 TCPWM LINE 模式), 开/关统一走 PWM:
     *   on=true  → PWM 100% 占空比 (恒定高电平, 阀导通)
     *   on=false → PWM 0%   占空比 (恒定低电平, 阀关断) */
    ValvePwm_SetOnOff(valve_id, on);
}

/* ========================================================================== */
/*  公开 API — 故障查询与通知                                                  */
/* ========================================================================== */

bool Bts724g_IsOpenFault(valve_id_t valve_id)
{
    if (valve_id >= VALVE_NUM_TOTAL) return false;
    return g_bts724g_fault_status.bts724g_fault_open[valve_id];
}

bool Bts724g_IsShortFault(valve_id_t valve_id)
{
    if (valve_id >= VALVE_NUM_TOTAL) return false;
    return g_bts724g_fault_status.bts724g_fault_short[valve_id];
}

void Bts724g_ReportExternalShort(valve_id_t valve_id, bool is_short)
{
    if (valve_id >= VALVE_NUM_TOTAL) return;

    g_bts724g_fault_status.bts724g_fault_short[valve_id] = is_short;

    if (is_short)
    {
        g_bts724g_fault_status.bts724g_fault_type[valve_id] = VALVE_FAULT_SHORT;
    }
    else if (!g_bts724g_fault_status.bts724g_fault_open[valve_id])
    {
        g_bts724g_fault_status.bts724g_fault_type[valve_id] = VALVE_FAULT_NONE;
    }
}

bool Bts724g_ReadST(valve_id_t valve_id)
{
    if (valve_id >= VALVE_NUM_TOTAL) return false;
    const bts724g_valve_map_t* m = &g_bts724g_valve_map[valve_id];
    return ReadST(m->chip_id, (m->ch_id < 2u) ? 0u : 1u);
}

void Bts724g_NotifyValveActive(valve_id_t valve_id, bool active)
{
    if (valve_id >= VALVE_NUM_TOTAL) return;
    s_valve_active[valve_id] = active;
}

bool Bts724g_IsValveActive(valve_id_t valve_id)
{
    if (valve_id >= VALVE_NUM_TOTAL) return false;
    return s_valve_active[valve_id];
}

/* ========================================================================== */
/*  诊断 — 阀开路检测 (单阀: OFF 态读 ST, 低 = 开路)                             */
/*                                                                            */
/*  ST 开漏低有效: ST=High 正常, ST=Low = 无电流 = 开路                          */
/*  有 partner: 开 partner → 读目标阀 OFF 态 ST → High=正常, Low=开路           */
/*  独占 ST:    开自身   → 读 ON 态 ST        → High=正常, Low=开路             */
/* ========================================================================== */
static void Diag_ValveOpenOne(valve_id_t vid)
{
    if (Bts724g_IsValveActive(vid)) return;

    const valve_id_t partner = s_st_partner[vid];
    bool is_fault = false;

    if (partner >= VALVE_NUM_TOTAL)
    {
        /* 独占 ST: 开自身 → 等 320μs → 读 ON 态 ST → High=正常, Low=开路 */
        bts724g_asr_lowside_enable(vid, 1u);
        Cy_SysTick_DelayInUs(LOWSIDE_SETTLE_US);
        Bts724g_SetValve(vid, true);
        Cy_SysTick_DelayInUs(VALVE_ON_SETTLE_US);
        is_fault = !Bts724g_ReadST(vid);  /* ST=Low → 无电流 → 开路 (BTS724G open-drain) */
        Bts724g_SetValve(vid, false);
        bts724g_asr_lowside_enable(vid, 0u);
    }
    else
    {
        /* 有 partner: 开 partner → 等 300μs → 读目标 OFF 态 ST → Low=正常, High=开路 */
        if (Bts724g_IsValveActive(partner)) return;
        if (g_bts724g_fault_status.bts724g_fault_short[partner]) return;

        bts724g_asr_lowside_enable(vid, 1u);
        bts724g_asr_lowside_enable(partner, 1u);
        Cy_SysTick_DelayInUs(LOWSIDE_SETTLE_US);
        Bts724g_SetValve(partner, true);
        Cy_SysTick_DelayInUs(VALVE_ON_SETTLE_US);
        is_fault = !Bts724g_ReadST(vid);  /* OFF态 ST=Low → partner 电流未拉低 → 目标线圈断 */
        Bts724g_SetValve(partner, false);
        bts724g_asr_lowside_enable(partner, 0u);
        bts724g_asr_lowside_enable(vid, 0u);
    }

    /* 3 次上下计数消抖 */
    if (is_fault)
    {
        if (s_open_candidate_count[vid] < FAULT_CONFIRM_NEED) s_open_candidate_count[vid]++;
        if (s_open_candidate_count[vid] >= FAULT_CONFIRM_NEED)
        {
            g_bts724g_fault_status.bts724g_fault_open[vid] = true;
            g_bts724g_fault_status.bts724g_fault_type[vid] = VALVE_FAULT_OPEN;
        }
    }
    else
    {
        if (s_open_candidate_count[vid] > 0u) s_open_candidate_count[vid]--;
        if (s_open_candidate_count[vid] == 0u)
        {
            g_bts724g_fault_status.bts724g_fault_open[vid] = false;
            if (g_bts724g_fault_status.bts724g_fault_type[vid] == VALVE_FAULT_OPEN)
                g_bts724g_fault_status.bts724g_fault_type[vid] = VALVE_FAULT_NONE;
        }
    }
}

/* ========================================================================== */
/*  诊断 — 阀短路检测 (单阀: 同组全关后仅开目标阀, FAR 高 = 对地短路)            */
/*                                                                            */
/*  同组全关 → 仅开 vid → 300μs → 读 FAR → High=短路, Low=正常                   */
/* ========================================================================== */
static void Diag_ValveShortOne(valve_id_t vid)
{
    if (Bts724g_IsValveActive(vid)) return;

    const short_group_id_t grp = s_bts724g_valve_to_short_group[vid];

    /* 同组有阀在工作则跳过 */
    for (uint8_t i = 0u; i < VALVE_NUM_TOTAL; i++)
        if (s_bts724g_valve_to_short_group[i] == grp && Bts724g_IsValveActive((valve_id_t)i)) return;

    /* 关闭同组所有阀 (安全兜底) */
    for (uint8_t i = 0u; i < VALVE_NUM_TOTAL; i++)
        if (s_bts724g_valve_to_short_group[i] == grp)
            Bts724g_SetValve((valve_id_t)i, false);

    bts724g_asr_lowside_enable(vid, 1u);
    Cy_SysTick_DelayInUs(LOWSIDE_SETTLE_US);
    Bts724g_SetValve(vid, true);
    Cy_SysTick_DelayInUs(VALVE_ON_SETTLE_US);

    const bool is_fault = ValveShort_ReadGroup(grp);  /* FAR=High → 短路 */

    Bts724g_SetValve(vid, false);
    bts724g_asr_lowside_enable(vid, 0u);

    /* 3 次上下计数消抖 */
    if (is_fault)
    {
        if (s_short_candidate_count[vid] < FAULT_CONFIRM_NEED) s_short_candidate_count[vid]++;
        if (s_short_candidate_count[vid] >= FAULT_CONFIRM_NEED)
        {
            g_bts724g_fault_status.bts724g_fault_short[vid] = true;
            g_bts724g_fault_status.bts724g_fault_type[vid]  = VALVE_FAULT_SHORT;
        }
    }
    else
    {
        if (s_short_candidate_count[vid] > 0u) s_short_candidate_count[vid]--;
        if (s_short_candidate_count[vid] == 0u)
        {
            g_bts724g_fault_status.bts724g_fault_short[vid] = false;
            if (g_bts724g_fault_status.bts724g_fault_type[vid] == VALVE_FAULT_SHORT)
                g_bts724g_fault_status.bts724g_fault_type[vid] = VALVE_FAULT_NONE;
        }
    }
}

/* ========================================================================== */
/*  诊断 — 芯片开路检测 (芯片级: 全 IN=HIGH, 两路 ST 均低 → 驱动级故障)          */
/*                                                                            */
/*  全部 IN=HIGH → 等 300μs → 两路 ST 均 Low → 芯片开路                         */
/*  防误报: 芯片上已有 ≥2 阀开路则跳过                                            */
/* ========================================================================== */
static void Diag_ChipOpenOne(uint8_t chip)
{
    const uint8_t num = g_chip_cfg[chip].num_valves;
    if (num == 0u) return;

    uint8_t open_cnt = 0u;
    for (uint8_t v = 0u; v < VALVE_NUM_TOTAL; v++)
    {
        const bts724g_valve_map_t* m = &g_bts724g_valve_map[v];
        if (m->chip_id != chip || m->ch_id >= num) continue;
        if (Bts724g_IsValveActive((valve_id_t)v)) return;
        if (g_bts724g_fault_status.bts724g_fault_type[v] == VALVE_FAULT_OPEN) open_cnt++;
    }
    if (open_cnt >= 2u) return;

    /* 全 IN=HIGH (走 PWM, IN 脚是 LINE 模式, GPIO 直写无效) → 读两路 ST */
    for (uint8_t v = 0u; v < VALVE_NUM_TOTAL; v++)
    {
        const bts724g_valve_map_t* m = &g_bts724g_valve_map[v];
        if (m->chip_id == chip && m->ch_id < num)
            Bts724g_SetValve((valve_id_t)v, true);
    }
    Cy_SysTick_DelayInUs(VALVE_ON_SETTLE_US);
    const bool st12_ok = ReadST(chip, 0u);
    const bool st34_ok = (g_chip_cfg[chip].st34_port != NULL) ? ReadST(chip, 1u) : true;
    for (uint8_t v = 0u; v < VALVE_NUM_TOTAL; v++)
    {
        const bts724g_valve_map_t* m = &g_bts724g_valve_map[v];
        if (m->chip_id == chip && m->ch_id < num)
            Bts724g_SetValve((valve_id_t)v, false);
    }

    /* 5 次消抖 */
    static uint8_t s_chip_open_s[5] = {0}, s_chip_ok_s[5] = {0};
    if (!st12_ok && !st34_ok)
    {
        s_chip_ok_s[chip] = 0u;
        if (s_chip_open_s[chip] < CHIP_FAULT_CONFIRM_NEED) s_chip_open_s[chip]++;
        if (s_chip_open_s[chip] >= CHIP_FAULT_CONFIRM_NEED)
            g_bts724g_fault_status.bts724g_chip_fault_type[chip] = VALVE_FAULT_OPEN;
    }
    else
    {
        s_chip_open_s[chip] = 0u;
        if (s_chip_ok_s[chip] < CHIP_FAULT_CONFIRM_NEED) s_chip_ok_s[chip]++;
        if (s_chip_ok_s[chip] >= CHIP_FAULT_CONFIRM_NEED)
            g_bts724g_fault_status.bts724g_chip_fault_type[chip] = VALVE_FAULT_NONE;
    }
}

/* ========================================================================== */
/*  诊断 — 芯片短路检测 (芯片级: 全 IN=LOW, FAR 为低 → 芯片内部短路)             */
/*                                                                            */
/*  全部 IN=LOW → 读 FAR → FAR=Low → 芯片内部短路                                 */
/* ========================================================================== */
static void Diag_ChipShortOne(uint8_t chip)
{
    const uint8_t num = g_chip_cfg[chip].num_valves;
    if (num == 0u) return;

    /* 守卫1: 本芯片/同 FAR 组有阀工作则跳过 */
    for (uint8_t v = 0u; v < VALVE_NUM_TOTAL; v++)
    {
        const bts724g_valve_map_t* m = &g_bts724g_valve_map[v];
        if (m->chip_id == chip && m->ch_id < num && Bts724g_IsValveActive((valve_id_t)v)) return;
    }
    const short_group_id_t grp = s_chip_short_group[chip];
    for (uint8_t v = 0u; v < VALVE_NUM_TOTAL; v++)
        if (s_bts724g_valve_to_short_group[v] == grp && Bts724g_IsValveActive((valve_id_t)v)) return;

    /* 守卫2: 本芯片有阀开路 → OUT 浮空 → FAR 不可信, 跳过 (防阀开路误报芯片短路) */
    for (uint8_t v = 0u; v < VALVE_NUM_TOTAL; v++)
    {
        const bts724g_valve_map_t* m = &g_bts724g_valve_map[v];
        if (m->chip_id == chip && m->ch_id < num
            && g_bts724g_fault_status.bts724g_fault_type[v] == VALVE_FAULT_OPEN) return;
    }

    /* 开低边: B 组芯片 (U12/U13/U19) 线圈地经低边, 必须开否则 OUT 浮空 */
    if (grp == SHORT_GROUP_B)
    {
        ASRFLowSideEnable(1u);                    /* P2.0: U12/U19 + U13 的 22口进 (原 FA_ASR) */
        if (chip == 3u) ASRRLowSideEnable(1u);    /* P6.2: U13 的 22口排 (原 DA_ASR) */
        Cy_SysTick_DelayInUs(LOWSIDE_SETTLE_US);
    }

    /* 全 IN=LOW (走 PWM) → 读 FAR → FAR=Low → 芯片内部短路 (FET 关不断, OUT 仍高) */
    for (uint8_t v = 0u; v < VALVE_NUM_TOTAL; v++)
    {
        const bts724g_valve_map_t* m = &g_bts724g_valve_map[v];
        if (m->chip_id == chip && m->ch_id < num)
            Bts724g_SetValve((valve_id_t)v, false);
    }
    Cy_SysTick_DelayInUs(CHIP_SETTLE_US);
    const bool is_short = !ValveShort_ReadGroup(grp);  /* FAR=Low → 短路 */

    /* 关低边 */
    if (grp == SHORT_GROUP_B)
    {
        ASRFLowSideEnable(0u);
        if (chip == 3u) ASRRLowSideEnable(0u);
    }

    /* 5 次消抖 */
    static uint8_t s_chip_short_s[5] = {0}, s_chip_noshort_s[5] = {0};
    if (is_short)
    {
        s_chip_noshort_s[chip] = 0u;
        if (s_chip_short_s[chip] < CHIP_FAULT_CONFIRM_NEED) s_chip_short_s[chip]++;
        if (s_chip_short_s[chip] >= CHIP_FAULT_CONFIRM_NEED)
            g_bts724g_fault_status.bts724g_chip_fault_type[chip] = VALVE_FAULT_SHORT;
    }
    else
    {
        s_chip_short_s[chip] = 0u;
        if (s_chip_noshort_s[chip] < CHIP_FAULT_CONFIRM_NEED) s_chip_noshort_s[chip]++;
        if (s_chip_noshort_s[chip] >= CHIP_FAULT_CONFIRM_NEED
            && g_bts724g_fault_status.bts724g_chip_fault_type[chip] == VALVE_FAULT_SHORT)
            g_bts724g_fault_status.bts724g_chip_fault_type[chip] = VALVE_FAULT_NONE;
    }
}

/* ========================================================================== */
/*  诊断主入口 (每 10ms 调用一次, 40ms 执行 1 步, 4 步轮转)                       */
/*                                                                            */
/*  4 步轮转: 开路 → 短路 → 芯片开路 → 芯片短路                                  */
/*  每步只测 1 个阀/芯片, 带 3 次对称消抖                                         */
/* ========================================================================== */
void ValveDiag_Process(void)
{
    /* 阀测试模式 (RTEfValWssTestForbit=1, 上层阀/WSS 测试): 诊断整体让路 —— 不驱动任何阀,
     * 也不开/关 P2.0、P6.2 低边 (低边由调用方 InLowSideSwX4_16/OutLowSideSwX2_16 自行控制),
     * 避免与上层的阀/低边测试互相踩踏。故障状态保持上次结果不变。 */
    if (RTEfValWssTestForbit != 0u) return;

    if (PSWvIgn < 180u || PSWvIgn > 320u) return;

    s_call_count++;
    if (s_call_count & 3u) return;   /* 每 4 次 (40ms) 执行 1 步 */

    static uint8_t s_step = 0u;
    static valve_id_t s_open_vid  = VALVE_RRI;
    static valve_id_t s_short_vid = VALVE_RRI;
    static uint8_t    s_chip_id   = 0u;

    switch (s_step)
    {
    case 0: /* 阀开路 */
        Diag_ValveOpenOne(s_open_vid);
        s_open_vid = (valve_id_t)((uint8_t)s_open_vid + 1u);
        if (s_open_vid >= VALVE_NUM_TOTAL) s_open_vid = VALVE_RRI;
        break;
    case 1: /* 阀短路 */
        Diag_ValveShortOne(s_short_vid);
        s_short_vid = (valve_id_t)((uint8_t)s_short_vid + 1u);
        if (s_short_vid >= VALVE_NUM_TOTAL) s_short_vid = VALVE_RRI;
        break;
    case 2: /* 芯片开路 */
        Diag_ChipOpenOne(s_chip_id);
        s_chip_id++;
        if (s_chip_id >= BTS724G_NUM_CHIPS) s_chip_id = 0u;
        break;
    case 3: /* 芯片短路 */
        Diag_ChipShortOne(s_chip_id);
        s_chip_id++;
        if (s_chip_id >= BTS724G_NUM_CHIPS) s_chip_id = 0u;
        break;
    }
    s_step = (s_step + 1u) & 3u;   /* 0→1→2→3 循环 */
}

/* ========================================================================== */
/*  调试支持                                                                   */
/* ========================================================================== */
#if (VALVE_DIAG_DEBUG != 0u)
volatile valve_diag_debug_t g_valve_diag_debug = {0};

void ValveDiag_DebugDump(void)
{
    g_valve_diag_debug.systick_ms = g_systick_ms;
    g_valve_diag_debug.far_raw_a  = ValveShort_ReadGroup(SHORT_GROUP_A) ? 1u : 0u;
    g_valve_diag_debug.far_raw_b  = ValveShort_ReadGroup(SHORT_GROUP_B) ? 1u : 0u;
    for (uint8_t i = 0u; i < VALVE_NUM_TOTAL; i++)
    {
        g_valve_diag_debug.active[i] = Bts724g_IsValveActive((valve_id_t)i) ? 1u : 0u;
    }
}
#endif
