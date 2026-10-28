/**
 * @file    gpio_pwm.c
 * @brief   GPIO 软件 PWM — ABS 阀 (12 路), 100μs 定时器中断驱动
 *
 * @details ABS 阀 (6 轮制动阀, 通电即打开) 改用 GPIO 位翻转模拟 PWM。
 *          时基 100μs 定时器 (TCPWM0_GRP0_CNT2), 中断里对 12 路 counter 比较,
 *          只在边沿 (SET/CLEAR 点) 写 GPIO, 其余节拍仅计数, 降低中断开销。
 *
 *          参数语义与硬件 PWM 一致 (period / cc0_start / htime, 单位 0.1ms):
 *            counter 0 → period-1 循环, [cc0_start, cc0_start+htime) 为高电平。
 */

#include "cy_project.h"
#include "cy_device_headers.h"
#include "gpio_pwm.h"
#include "peri_div.h"
#include "cmn.h"

/* ========================================================================== */
/*  1. 硬件宏                                                                   */
/*                                                                            */
/*  实例: TCPWM0_GRP0_CNT2 (比较模式, 连续计数)                                */
/*  时钟: clk_peri → ÷N(动态) → 1MHz (DIVBY_1)                                */
/*  周期: 100 - 1 = 99,  99+1 us = 100μs                                      */
/* ========================================================================== */
#define GPIOPWM_TIMER_CH        TCPWM0_GRP0_CNT2
#define GPIOPWM_CLK_SRC         PCLK_TCPWM0_CLOCKS2
#define GPIOPWM_IRQ_SRC         tcpwm_0_interrupts_2_IRQn
#define GPIOPWM_CLK_DIV         DIV16_NO_GPIOPWM   /* 分频器 #5, 1MHz */
#define GPIOPWM_CLK_TARGET_HZ   1000000ul          /* 1MHz */
#define GPIOPWM_TICKS           100u               /* 100μs = 100 ticks @ 1MHz */

/* ========================================================================== */
/*  2. 12 路 GPIO 表 (ABS 阀 = 6 轮制动 × 进/排)                                */
/*    顺序与下方 s_gpio_pwm_valves 映射一致                                     */
/* ========================================================================== */
typedef struct
{
    volatile stc_GPIO_PRT_t* port;
    uint32_t                 pin;
} gpio_pwm_pin_t;

static const gpio_pwm_pin_t s_gpio_pin[GPIO_PWM_CH_NUM] =
{
    { GPIO_PRT22, 2u },   /*  0 RRI 后桥右进 (P22.2) */
    { GPIO_PRT22, 1u },   /*  1 RRO 后桥右排 (P22.1) */
    { GPIO_PRT22, 0u },   /*  2 LRO 后桥左排 (P22.0) */
    { GPIO_PRT21, 5u },   /*  3 LRI 后桥左进 (P21.5) */
    { GPIO_PRT0,  0u },   /*  4 RFI 前桥右进 (P0.0)  */
    { GPIO_PRT23, 4u },   /*  5 RFO 前桥右排 (P23.4) */
    { GPIO_PRT23, 3u },   /*  6 LFO 前桥左排 (P23.3) */
    { GPIO_PRT22, 3u },   /*  7 LFI 前桥左进 (P22.3) */
    { GPIO_PRT21, 1u },   /*  8 RXI 辅助右进 (P21.1) */
    { GPIO_PRT21, 0u },   /*  9 RXO 辅助右排 (P21.0) */
    { GPIO_PRT19, 3u },   /* 10 LXO 辅助左排 (P19.3) */
    { GPIO_PRT19, 2u },   /* 11 LXI 辅助左进 (P19.2) */
};

/* valve_id → 通道 idx 映射 (ABS 阀 12 路, 与 s_gpio_pin 表顺序一致) */
static const valve_id_t s_gpio_pwm_valves[GPIO_PWM_CH_NUM] =
{
    VALVE_RRI, VALVE_RRO, VALVE_LRO, VALVE_LRI,
    VALVE_RFI, VALVE_RFO, VALVE_LFO, VALVE_LFI,
    VALVE_RXI, VALVE_RXO, VALVE_LXO, VALVE_LXI,
};

/** @brief valve_id → 通道号 (0~11), 非 ABS 阀返回 -1 */
static int8_t GpioPwm_Idx(valve_id_t vid)
{
    for (uint8_t i = 0u; i < GPIO_PWM_CH_NUM; i++)
    {
        if (s_gpio_pwm_valves[i] == vid) return (int8_t)i;
    }
    return -1;
}

/* ========================================================================== */
/*  3. 通道状态                                                                 */
/* ========================================================================== */
typedef struct
{
    uint16_t period;     /* 周期 (0.1ms 节拍数), 0 = 通道关闭 */
    uint16_t cc0_start;  /* 高电平起点 (SET 点) */
    uint16_t htime;      /* 高电平宽度 */
    uint16_t counter;    /* 当前计数 */
    bool     suspended;  /* 诊断让路标志 (SetOnOff 直写期间挂起) */
    bool     level;      /* 当前输出电平 (边沿检测缓存) */
    /* 低电平段改参数时缓存, 周期边界才生效 (对齐硬件 SetDutyLive 的 was_high 语义) */
    uint16_t pend_period;
    uint16_t pend_cc0_start;
    uint16_t pend_htime;
    bool     has_pend;
} gpio_pwm_ch_t;

static gpio_pwm_ch_t s_ch[GPIO_PWM_CH_NUM];

/* ========================================================================== */
/*  4. 100μs 定时器配置                                                        */
/* ========================================================================== */
static const cy_stc_tcpwm_counter_config_t gpioPwmCfg =
{
    .period            = GPIOPWM_TICKS - 1u,
    .clockPrescaler    = CY_TCPWM_PRESCALER_DIVBY_1,
    .runMode           = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection    = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause       = 0u,
    .compareOrCapture  = CY_TCPWM_COUNTER_MODE_COMPARE,
    .compare0 = 0, .compare0_buff = 0, .compare1 = 0, .compare1_buff = 0,
    .enableCompare0Swap = false, .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_ON_TC,
    .capture0InputMode  = CY_TCPWM_INPUT_LEVEL,  .capture0Input = 0u,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,  .reloadInput = 0u,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,  .startInput  = 0u,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,  .stopInput   = 0u,
    .capture1InputMode  = CY_TCPWM_INPUT_LEVEL,  .capture1Input = 0u,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,  .countInput   = 1u,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_OVERFLOW,
    .trigger1EventCfg   = CY_TCPWM_COUNTER_OVERFLOW,
};

/* ========================================================================== */
/*  5. 节拍推进 (100μs 中断调用)                                                */
/* ========================================================================== */
static void GpioPwm_Tick(void)
{
    for (uint8_t i = 0u; i < GPIO_PWM_CH_NUM; i++)
    {
        gpio_pwm_ch_t *c = &s_ch[i];
        if (c->period == 0u || c->suspended) continue;

        c->counter++;
        if (c->counter >= c->period)
        {
            c->counter = 0u;
            /* 周期边界: 应用 pending (低电平段改的参数, 下周期才生效) */
            if (c->has_pend)
            {
                c->period    = c->pend_period;
                c->cc0_start = c->pend_cc0_start;
                c->htime     = c->pend_htime;
                c->has_pend  = false;
            }
        }

        /* 高电平区间 [cc0_start, cc0_start + htime), 越界恒高 */
        uint16_t cc1 = (uint16_t)(c->cc0_start + c->htime);
        bool on = (c->htime != 0u) &&
                  ((cc1 > c->period) ? true
                   : (c->counter >= c->cc0_start && c->counter < cc1));

        /* 只在边沿写 GPIO, 其余节拍仅计数 */
        if (on != c->level)
        {
            c->level = on;
            Cy_GPIO_Write(s_gpio_pin[i].port, s_gpio_pin[i].pin, on ? 1ul : 0ul);
        }
    }
}

/* ========================================================================== */
/*  6. 中断 (优先级 2, 低于 CAN0/CAN1 的 0 和 10ms 定时器的 1)                  */
/* ========================================================================== */
static void GpioPwm_Handler(void)
{
    Cy_Tcpwm_ClearInterrupt(GPIOPWM_TIMER_CH, CY_TCPWM_INT_ON_TC);
    GpioPwm_Tick();
}

/** @brief 中断向量入口 (转调 GpioPwm_Handler) */
void GpioPwm_ISR(void)
{
    GpioPwm_Handler();
}

/* ========================================================================== */
/*  7. 初始化                                                                   */
/* ========================================================================== */
/** @brief 100μs 定时器初始化 + 中断挂载 (12 路引脚 GPIO 化由 ValvePwm_Init 完成)
 */
void GpioPwm_Init(void)
{
    periph_divider(GPIOPWM_CLK_SRC, CY_SYSCLK_DIV_16_BIT, GPIOPWM_CLK_DIV, GPIOPWM_CLK_TARGET_HZ);

    Cy_Tcpwm_Counter_Init(GPIOPWM_TIMER_CH, &gpioPwmCfg);
    Cy_Tcpwm_Counter_Enable(GPIOPWM_TIMER_CH);
    Cy_Tcpwm_TriggerStart(GPIOPWM_TIMER_CH);

    cy_stc_sysint_irq_t irqCfg = {.sysIntSrc = GPIOPWM_IRQ_SRC, .intIdx = CPUIntIdx4_IRQn, .isEnabled = true};
    Cy_SysInt_InitIRQ(&irqCfg);
    Cy_SysInt_SetSystemIrqVector(irqCfg.sysIntSrc, GpioPwm_ISR);
    NVIC_SetPriority(irqCfg.intIdx, 2u);
    NVIC_ClearPendingIRQ(irqCfg.intIdx);
    NVIC_EnableIRQ(irqCfg.intIdx);
}

/* ========================================================================== */
/*  8. 对外接口                                                                 */
/* ========================================================================== */
/** @brief 设置 PWM 参数 (0.1ms 单位) — 幂等短路 + 相位连续 + was_high 平滑过渡
 *  @note  非法参数 (period=0 / cc0_start>=period) 忽略;
 *         高电平段改 → 立即生效(延长/截断); 低电平段改 → 下周期生效(缓存 pending)
 */
void GpioPwm_SetDuty(valve_id_t vid, uint16_t period, uint16_t cc0_start, uint16_t htime)
{
    int8_t idx = GpioPwm_Idx(vid);
    if (idx < 0) return;

    gpio_pwm_ch_t *c = &s_ch[idx];

    /* 非法参数 → 忽略 (与硬件 SetDutyLive 一致) */
    if (period == 0u || cc0_start >= period) return;

    /* 幂等短路: 参数未变 (无 pending 且非诊断挂起) → 不动作, 避免反复腰斩波形 */
    if (c->suspended == false && c->has_pend == false &&
        c->period == period && c->cc0_start == cc0_start && c->htime == htime)
    {
        return;
    }

    /* 判断是否立即生效 (对齐硬件 SetDutyLive 的 was_high 语义) */
    bool immediate;
    if (c->suspended || c->period == 0u)
    {
        /* 诊断恢复 / 首次: 无旧周期 → 立即生效 + 从头开始 */
        immediate    = true;
        c->suspended = false;
        c->counter   = 0u;
    }
    else
    {
        /* 改前是否在高电平段 (与 Tick 的 on 判断同式) */
        uint16_t old_cc1 = (uint16_t)(c->cc0_start + c->htime);
        immediate = (c->htime != 0u) &&
                    ((old_cc1 > c->period) ? (c->counter >= c->cc0_start)
                     : (c->counter >= c->cc0_start && c->counter < old_cc1));
    }

    if (immediate)
    {
        /* 高电平段: 立即生效, counter 不动 → 当前高电平无缝延长/截断 */
        c->period    = period;
        c->cc0_start = cc0_start;
        c->htime     = htime;
        c->has_pend  = false;
    }
    else
    {
        /* 低电平段: 缓存, 本周期走完, 下周期才生效 (防周期中途突然翻高) */
        c->pend_period    = period;
        c->pend_cc0_start = cc0_start;
        c->pend_htime     = htime;
        c->has_pend       = true;
    }
}

/** @brief 直接开/关 (诊断/上电直写), 挂起软件 PWM 由本函数接管输出
 */
void GpioPwm_SetOnOff(valve_id_t vid, bool on)
{
    int8_t idx = GpioPwm_Idx(vid);
    if (idx < 0) return;

    gpio_pwm_ch_t *c = &s_ch[idx];
    c->suspended = true;
    c->level     = on;
    Cy_GPIO_Write(s_gpio_pin[idx].port, s_gpio_pin[idx].pin, on ? 1ul : 0ul);
}
