/**
 * @file    wheel_speed.c
 * @brief   六通道轮速采集实现 — TCPWM 捕获 + PDMA 搬运 + 时间差法 + 低速插值
 *
 * @details 1) TCPWM 捕获 CC0(上升沿)/CC1(下降沿), CC1_MATCH 经 Trigger-MUX
 *             触发 PDMA, 硬件把 CC0/CC1 搬进环形缓冲 (每脉冲 2 个 word);
 *          2) 每 10ms 主循环取走新增脉冲, 末脉冲与上一周期时间差算速度;
 *          3) 脉宽 (CC1-CC0) < PWM_CAP_250US 的脉冲判为毛刺丢弃;
 *          4) 低速无脉冲时按等级插值, 避免速度阶梯跳变。
 *
 * @note    时钟: PERI_CLK 40MHz → DIV16_NO_WHEEL(2MHz) → TCPWM ÷2 → 1MHz(1us)
 *          缓冲布局: g_pdmaDestBuffer[i][0..29]=CC0, [30..59]=CC1 (2D DMA)
 */

#include "wheel_speed.h"
#include "cmn.h"
#include "RTE.h"
#include "cy_project.h"
#include "cy_device_headers.h"

/* ========================================================================== */
/*  类型定义                                                                   */
/* ========================================================================== */

/** @brief 每通道算法状态 + 输出指针 */
typedef struct
{
    uint8_t   notMinSpeedFlag;   /* 上一周期是否为最小速度 0=最小 1=非最小 */
    uint8_t   insertWait;        /* 插值等级 (1~4)                          */
    uint8_t   waitForCalc;       /* 插值进度                                */
    uint32_t  startCnt;          /* 起始脉冲的时钟数                        */
    uint32_t  lastCnt;           /* 最新脉冲的时钟数                        */
    uint16_t *wheelSpeed;        /* 输出: 指向 g_wheel_speed_rpm[indx]      */
} wheel_speed_data_t;

/** @brief 单通道完整硬件资源 */
typedef struct
{
    volatile stc_TCPWM_GRP_CNT_t *tcpwm;    /* TCPWM 实例         */
    en_clk_dst_t                  clkDest;  /* 外设时钟目标       */
    volatile stc_GPIO_PRT_t      *port;     /* 捕获引脚 port      */
    uint32_t                      pin;      /* 捕获引脚 pin       */
    en_hsiom_sel_t                hsiom;    /* 引脚复用           */
    volatile stc_DW_t            *pDma;     /* PDMA 实例          */
    uint32_t                      dmaChn;   /* PDMA 通道号        */
    uint32_t                      inTrig;   /* Trigger-MUX 输入   */
    uint32_t                      outTrig;  /* Trigger-MUX 输出   */
} ws_ch_cfg_t;

/* ========================================================================== */
/*  换算宏 (时间差法)                                                           */
/*                                                                            */
/*  v = PI × D / teeth × c / t × 1e6  [mm/s]                                   */
/*    D = RTEVWRDiameter (单位 1mm), t 单位 us, c = 脉冲个数                     */
/* ========================================================================== */

static uint16_t ws_vss(uint16_t c, uint32_t t)
{
    if (RTEVWRDiameter == 0u)
    {
        return 0xFFFFu;
    }

    /* 用 float(单精度) 而非 float64_t(double): CM4 的 VFPv4_sp 只带单精度硬件,
     * 写成 double 会拉入 m7M_tls 整套软件双精度库(约 1.1KB flash)。
     * 轮速只需 ~7 位有效数字, 单精度足够。 */
    float toothCnt;
    if (RTEVWSToothCnt == 0u)
    {
        toothCnt = 80.0f;
    }
    else if (RTEVWSToothCnt == 1u)
    {
        toothCnt = 100.0f;
    }
    else if (RTEVWSToothCnt == 2u)
    {
        toothCnt = 120.0f;
    }
    else
    {
        return 0xFFFFu;
    }

    return (uint16_t)((float)PI * (float)RTEVWRDiameter / toothCnt
                      * (float)c / (float)t * 1000000.0f);
}

/* 低速插值分级阈值 (10ms/20ms/30ms/40ms 一个脉冲对应的速度) */
#define _0msPulsSPEED     0u
#define _10msPulsSPEED    (ws_vss(1u, 10000u))
#define _20msPulsSPEED    (ws_vss(1u, 20000u))
#define _30msPulsSPEED    (ws_vss(1u, 30000u))
#define _40msPulsSPEED    (ws_vss(1u, 40000u))

/* ========================================================================== */
/*  全局 / 静态数据                                                            */
/* ========================================================================== */

uint16_t g_wheel_speed_rpm[NUM_CAPTURE_CHANNELS] = {0u};

/* 通道资源表 (顺序 = g_wheel_speed_rpm 索引 = psw_data 物理通道) */
static const ws_ch_cfg_t g_ws_ch[NUM_CAPTURE_CHANNELS] =
{
    { CH0_TCPWM, CH0_PCLK, CH0_PORT, CH0_PIN, CH0_MUX, DW0, CH0_DMA_CHN, CH0_DMA_TRGI, CH0_DMA_TRGO },
    { CH1_TCPWM, CH1_PCLK, CH1_PORT, CH1_PIN, CH1_MUX, DW0, CH1_DMA_CHN, CH1_DMA_TRGI, CH1_DMA_TRGO },
    { CH2_TCPWM, CH2_PCLK, CH2_PORT, CH2_PIN, CH2_MUX, DW0, CH2_DMA_CHN, CH2_DMA_TRGI, CH2_DMA_TRGO },
    { CH3_TCPWM, CH3_PCLK, CH3_PORT, CH3_PIN, CH3_MUX, DW0, CH3_DMA_CHN, CH3_DMA_TRGI, CH3_DMA_TRGO },
    { CH4_TCPWM, CH4_PCLK, CH4_PORT, CH4_PIN, CH4_MUX, DW0, CH4_DMA_CHN, CH4_DMA_TRGI, CH4_DMA_TRGO },
    { CH5_TCPWM, CH5_PCLK, CH5_PORT, CH5_PIN, CH5_MUX, DW0, CH5_DMA_CHN, CH5_DMA_TRGI, CH5_DMA_TRGO },
};

/* PDMA 环形缓冲: 前 TRIG_MAX_COUNT_PER_10MS 存 CC0, 后 30 存 CC1 */
static uint32_t g_pdmaDestBuffer  [NUM_CAPTURE_CHANNELS][TRIG_MAX_COUNT_PER_10MS * 2];
static uint32_t g_pdmaDestBufferCc0[NUM_CAPTURE_CHANNELS][TRIG_MAX_COUNT_PER_10MS];
static uint32_t g_pdmaDestBufferCc1[NUM_CAPTURE_CHANNELS][TRIG_MAX_COUNT_PER_10MS];

static uint8_t g_readIndx   [NUM_CAPTURE_CHANNELS];   /* 上次读到 CC0 的位置 */
static uint8_t g_readDataCnt[NUM_CAPTURE_CHANNELS];   /* 本次新增脉冲数       */

static cy_stc_pdma_descr_t g_descrList[NUM_CAPTURE_CHANNELS];   /* DMA 描述符 */

/* 算法状态 (wheelSpeed 指向输出数组) */
static wheel_speed_data_t g_ws_record[NUM_CAPTURE_CHANNELS] =
{
    { .wheelSpeed = &g_wheel_speed_rpm[0] },
    { .wheelSpeed = &g_wheel_speed_rpm[1] },
    { .wheelSpeed = &g_wheel_speed_rpm[2] },
    { .wheelSpeed = &g_wheel_speed_rpm[3] },
    { .wheelSpeed = &g_wheel_speed_rpm[4] },
    { .wheelSpeed = &g_wheel_speed_rpm[5] },
};

/* ========================================================================== */
/*  TCPWM 捕获配置 (CC0=上升沿, CC1=下降沿, CC1 匹配触发 DMA)                    */
/* ========================================================================== */

static const cy_stc_tcpwm_counter_config_t g_capCounterCfg =
{
    .period             = PWM_CAP_PERIOD_TICKS,
    .clockPrescaler     = PWM_CAP_PRESCALER,
    .runMode            = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection     = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause        = false,
    .compareOrCapture   = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0           = 0ul,
    .compare0_buff      = 0ul,
    .compare1           = 0ul,
    .compare1_buff      = 0ul,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .capture0InputMode  = CY_TCPWM_INPUT_RISING_EDGE,
    .capture0Input      = 2ul,
    .capture1InputMode  = CY_TCPWM_INPUT_FALLING_EDGE,
    .capture1Input      = 2ul,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0ul,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0ul,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,
    .stopInput          = 0ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1ul,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_CC1_MATCH,   /* 一个完整脉冲后触发 DMA */
    .trigger1EventCfg   = CY_TCPWM_COUNTER_CC0_MATCH,
};

/* 引脚配置模板 (hsiom 在运行期按通道填入) */
static cy_stc_gpio_pin_config_t g_capPinCfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom     = HSIOM_SEL_GPIO,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/* ========================================================================== */
/*  PDMA 初始化 (每通道: 2D 描述符搬 CC0+CC1 → 环形缓冲, 自链循环)               */
/* ========================================================================== */
static void ws_dma_init(uint8_t indx)
{
    const ws_ch_cfg_t *cfg = &g_ws_ch[indx];

    cy_stc_pdma_descr_config_t descrCfg =
    {
        .deact          = 0ul,                              /* 不等待触发失效 */
        .intrType       = CY_PDMA_INTR_DESCRCHAIN_CMPLT,
        .trigoutType    = CY_PDMA_TRIGOUT_DESCRCHAIN_CMPLT,
        .chStateAtCmplt = CY_PDMA_CH_ENABLED,
        .triginType     = CY_PDMA_TRIGIN_XLOOP,             /* 每脉冲触发一次 */
        .dataSize       = CY_PDMA_WORD,
        .srcTxfrSize    = CY_PDMA_WORD,
        .destTxfrSize   = CY_PDMA_WORD,
        .descrType      = CY_PDMA_2D_TRANSFER,
        .srcAddr        = (void *)&cfg->tcpwm->unCC0.u32Register,
        .destAddr       = (void *)&g_pdmaDestBuffer[indx][0],
        .srcXincr       = 2,                                /* CC0 → CC1 跨 2 word */
        .destXincr      = (int32_t)TRIG_MAX_COUNT_PER_10MS, /* CC0[i] / CC1[30+i]  */
        .xCount         = 2u,                               /* 搬 CC0 + CC1        */
        .srcYincr       = 0,
        .destYincr      = 1,
        .yCount         = TRIG_MAX_COUNT_PER_10MS,
        .descrNext      = &g_descrList[indx],               /* 自链, 无限循环      */
    };

    cy_stc_pdma_chnl_config_t chnlCfg =
    {
        .PDMA_Descriptor = &g_descrList[indx],
        .preemptable     = 0ul,
        .priority        = 0ul,
        .enable          = 1ul,                             /* 初始化后即使能 */
    };

    Cy_PDMA_Disable(cfg->pDma);
    Cy_PDMA_Chnl_DeInit(cfg->pDma, cfg->dmaChn);
    (void)Cy_PDMA_Descr_Init(&g_descrList[indx], &descrCfg);
    (void)Cy_PDMA_Chnl_Init(cfg->pDma, cfg->dmaChn, &chnlCfg);
    Cy_PDMA_Chnl_Enable(cfg->pDma, cfg->dmaChn);
    Cy_PDMA_Enable(cfg->pDma);

    /* 打通 Trigger-MUX: TCPWM tr_out → mux3 → PDMA0 tr_in */
    (void)Cy_TrigMux_Connect(cfg->inTrig, cfg->outTrig,
                             CY_TR_MUX_TR_INV_DISABLE, TRIGGER_TYPE_EDGE, 0ul);
}

/* ========================================================================== */
/*  单通道初始化 (时钟 + 引脚 + DMA + TCPWM)                                    */
/* ========================================================================== */
static void ws_cap_init(uint8_t indx)
{
    const ws_ch_cfg_t *cfg = &g_ws_ch[indx];

    /* 时钟: 分频器 assign 到本通道 TCPWM + 设为 2MHz */
    periph_divider(cfg->clkDest, CY_SYSCLK_DIV_16_BIT, DIV16_NO_WHEEL,
                   CAPTURE_CLK_TARGET_HZ);

    /* 捕获引脚 */
    g_capPinCfg.hsiom = cfg->hsiom;
    Cy_GPIO_Pin_Init(cfg->port, cfg->pin, &g_capPinCfg);

    /* DMA */
    ws_dma_init(indx);

    /* TCPWM 捕获 */
    Cy_Tcpwm_Counter_Init(cfg->tcpwm, &g_capCounterCfg);
    Cy_Tcpwm_Counter_Enable(cfg->tcpwm);
    Cy_Tcpwm_TriggerStart(cfg->tcpwm);
}

/* ========================================================================== */
/*  DMA 数据读取 — 取出本周期新增脉冲 (处理环形回绕)                             */
/* ========================================================================== */
static void ws_dma_read_new(uint8_t indx)
{
    const ws_ch_cfg_t *cfg = &g_ws_ch[indx];
    const uint8_t readIndx = g_readIndx[indx];
    const uint8_t curIndx  = (uint8_t)cfg->pDma->CH_STRUCT[cfg->dmaChn].unCH_IDX.stcField.u8Y_IDX;
    uint8_t i;

    if (curIndx == readIndx)
    {
        g_readDataCnt[indx] = 0u;
    }
    else if (curIndx > readIndx)
    {
        g_readDataCnt[indx] = (uint8_t)(curIndx - readIndx);
        for (i = 0u; i < g_readDataCnt[indx]; i++)
        {
            g_pdmaDestBufferCc0[indx][i] = g_pdmaDestBuffer[indx][readIndx + i];
            g_pdmaDestBufferCc1[indx][i] = g_pdmaDestBuffer[indx][TRIG_MAX_COUNT_PER_10MS + readIndx + i];
        }
    }
    else
    {
        /* 回绕: [readIndx..29] + [0..curIndx] */
        const uint8_t len1 = (uint8_t)(TRIG_MAX_COUNT_PER_10MS - readIndx);
        const uint8_t len2 = curIndx;
        g_readDataCnt[indx] = (uint8_t)(len1 + len2);

        for (i = 0u; i < len1; i++)
        {
            g_pdmaDestBufferCc0[indx][i] = g_pdmaDestBuffer[indx][readIndx + i];
            g_pdmaDestBufferCc1[indx][i] = g_pdmaDestBuffer[indx][TRIG_MAX_COUNT_PER_10MS + readIndx + i];
        }
        for (i = 0u; i < len2; i++)
        {
            g_pdmaDestBufferCc0[indx][i + len1] = g_pdmaDestBuffer[indx][i];
            g_pdmaDestBufferCc1[indx][i + len1] = g_pdmaDestBuffer[indx][TRIG_MAX_COUNT_PER_10MS + i];
        }
    }

    g_readIndx[indx] = curIndx;
}

/* ========================================================================== */
/*  单通道轮速计算 (时间差法 + 脉宽过滤 + 低速插值)                           */
/* ========================================================================== */
static void ws_calc_by_index(uint8_t indx)
{
    uint32_t wheel_tTime = 0u;
    uint32_t data_buf[TRIG_MAX_COUNT_PER_10MS] = {0};
    uint8_t  dataCount = 0u;
    uint8_t  i;

    /* 1. 脉宽过滤: 仅保留 CC1-CC0 > PWM_CAP_250US 的有效脉冲, 记录其 CC0 */
    for (i = 0u; i < g_readDataCnt[indx]; i++)
    {
        const uint32_t cc0 = g_pdmaDestBufferCc0[indx][i];
        const uint32_t cc1 = g_pdmaDestBufferCc1[indx][i];

        if (((cc1 >= cc0) && ((cc1 - cc0) > PWM_CAP_250US)) ||
            ((cc1 <  cc0) && ((cc1 + PWM_CAP_PERIOD_TICKS + 1u - cc0) > PWM_CAP_250US)))
        {
            data_buf[dataCount] = cc0;
            dataCount++;
        }
    }

    if (dataCount > 0u)
    {
        g_ws_record[indx].waitForCalc = 0u;

        /* 上一个周期为最小速度: 只记录起始时间点 */
        if (g_ws_record[indx].notMinSpeedFlag == 0u)
        {
            g_ws_record[indx].startCnt = data_buf[dataCount - 1u];
            g_ws_record[indx].notMinSpeedFlag = 1u;
        }
        else
        {
            g_ws_record[indx].lastCnt = data_buf[dataCount - 1u];

            if (g_ws_record[indx].lastCnt > g_ws_record[indx].startCnt)
            {
                wheel_tTime = g_ws_record[indx].lastCnt - g_ws_record[indx].startCnt;
            }
            else
            {
                wheel_tTime = (PWM_CAP_PERIOD_TICKS + 1u)
                            + g_ws_record[indx].lastCnt - g_ws_record[indx].startCnt;
            }
            g_ws_record[indx].startCnt = g_ws_record[indx].lastCnt;

            if (wheel_tTime > 4000u)
            {
                if (wheel_tTime < 50000u)
                {
                    *(g_ws_record[indx].wheelSpeed) = ws_vss(dataCount, wheel_tTime);
                }
                else
                {
                    *(g_ws_record[indx].wheelSpeed) = _0msPulsSPEED;
                }

                /* 低速插值等级 */
                if      (*(g_ws_record[indx].wheelSpeed) > _10msPulsSPEED) g_ws_record[indx].insertWait = 1u;
                else if (*(g_ws_record[indx].wheelSpeed) > _20msPulsSPEED) g_ws_record[indx].insertWait = 2u;
                else if (*(g_ws_record[indx].wheelSpeed) > _30msPulsSPEED) g_ws_record[indx].insertWait = 3u;
                else if (*(g_ws_record[indx].wheelSpeed) > _40msPulsSPEED) g_ws_record[indx].insertWait = 4u;
                else                                                       g_ws_record[indx].insertWait = 0u;
            }
        }
    }
    else
    {
        /* 本周期无有效脉冲 */
        g_ws_record[indx].waitForCalc++;
        if (g_ws_record[indx].waitForCalc < 5u)
        {
            /* 插值: 用实时计数器补一个速度, 平滑下降 */
            if (g_ws_record[indx].waitForCalc == g_ws_record[indx].insertWait)
            {
                g_ws_record[indx].lastCnt = Cy_Tcpwm_Counter_GetCounter(g_ws_ch[indx].tcpwm);

                if (g_ws_record[indx].lastCnt > g_ws_record[indx].startCnt)
                {
                    wheel_tTime = g_ws_record[indx].lastCnt - g_ws_record[indx].startCnt;
                }
                else
                {
                    wheel_tTime = (PWM_CAP_PERIOD_TICKS + 1u)
                                + g_ws_record[indx].lastCnt - g_ws_record[indx].startCnt;
                }

                if (wheel_tTime < 50000u)
                {
                    const uint16_t aa = ws_vss(1u, wheel_tTime);
                    switch (g_ws_record[indx].waitForCalc)
                    {
                    case 1u: if (aa < _10msPulsSPEED) *(g_ws_record[indx].wheelSpeed) = aa; break;
                    case 2u: if (aa < _20msPulsSPEED) *(g_ws_record[indx].wheelSpeed) = aa; break;
                    case 3u: if (aa < _30msPulsSPEED) *(g_ws_record[indx].wheelSpeed) = aa; break;
                    case 4u: if (aa < _40msPulsSPEED) *(g_ws_record[indx].wheelSpeed) = aa; break;
                    default: break;
                    }
                }
                else
                {
                    *(g_ws_record[indx].wheelSpeed) = _0msPulsSPEED;
                }
                g_ws_record[indx].insertWait++;
            }
        }
        else
        {
            /* 插值完成 (5 × 10ms = 50ms 无脉冲), 判零 */
            g_ws_record[indx].notMinSpeedFlag = 0u;
            g_ws_record[indx].waitForCalc     = 0u;
            g_ws_record[indx].insertWait      = 0u;
            *(g_ws_record[indx].wheelSpeed)   = _0msPulsSPEED;
        }
    }
}

/* ========================================================================== */
/*  对外: 轮速计算 — 主循环每 10ms 调用                                         */
/* ========================================================================== */
void WheelSpeed_CalcAndSend(void)
{
    uint8_t i;

    /* 1. 取出各通道本周期新增脉冲 */
    for (i = 0u; i < NUM_CAPTURE_CHANNELS; i++)
    {
        ws_dma_read_new(i);
    }

    /* 2. 逐通道计算轮速 (结果写入 g_wheel_speed_rpm[]) */
    for (i = 0u; i < NUM_CAPTURE_CHANNELS; i++)
    {
        ws_calc_by_index(i);
    }
}

/* ========================================================================== */
/*  整车参考车速: 六通道中值 (mm/s) — adc 间隙检测门控用                        */
/* ========================================================================== */
uint16_t WheelSpeed_VehicleRefRpm(void)
{
    uint16_t sorted[NUM_CAPTURE_CHANNELS] = {
        g_wheel_speed_rpm[0], g_wheel_speed_rpm[1], g_wheel_speed_rpm[2],
        g_wheel_speed_rpm[3], g_wheel_speed_rpm[4], g_wheel_speed_rpm[5]
    };
    uint8_t j, k;

    for (j = 0u; j < (NUM_CAPTURE_CHANNELS - 1u); j++)
    {
        for (k = (uint8_t)(j + 1u); k < NUM_CAPTURE_CHANNELS; k++)
        {
            if (sorted[j] > sorted[k])
            {
                const uint16_t tmp = sorted[j];
                sorted[j] = sorted[k];
                sorted[k] = tmp;
            }
        }
    }

    return (uint16_t)(((uint32_t)sorted[2] + sorted[3]) / 2u);
}

/* ========================================================================== */
/*  初始化入口: 六通道统一初始化                                                */
/* ========================================================================== */
void WheelSpeed_Init(void)
{
    uint8_t i;

    for (i = 0u; i < NUM_CAPTURE_CHANNELS; i++)
    {
        ws_cap_init(i);
    }
}
