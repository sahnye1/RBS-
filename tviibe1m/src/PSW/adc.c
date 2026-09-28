/**
 * @file    adc.c
 * @brief   ADC 采集集中实现 — SAR0 九通道一个 Group + SAR1 (P12.1)
 *
 * @details 原先分散在 sensor_diag.c (轮速诊断 CH0~CH5) / sar1_adc.c (压力 CH6~CH7 +
 *          VPOWER CH8) / p12_1_adc.c (独立 SAR1) 三个模块, 现合并到本文件统一管理。
 *
 *          每 10ms 的采集流程 (Adc_Read):
 *            ① 软件触发 SAR0 组头 CH0 → 硬件自动序列 CH0~CH8 → 轮询组尾 CH8 grpDone
 *               (带超时保护, 超时则本轮作废并累加 Adc_GetTimeoutCount())
 *            ② 读 CH0~CH5 → 存入 100 点环形缓冲 (供轮速传感器诊断)
 *            ③ 读 CH6~CH8 → 压力换算/故障消抖 + VPOWER 换算/报警消抖
 *            ④ 触发 P12.1 (独立 SAR1 实例) → 读回原始值/mV
 *
 *          诊断判决 (Adc_Process, 必须在轮速计算之后调用):
 *            缓冲满 100 点 (1s) 后按平均值/峰峰值做滞回判决, 间隙诊断用车速门控,
 *            结果写入 g_sensor_diag[] / g_sensor_fault_status。
 */

#include "adc.h"
#include "psw_data.h"        /* PSWvIgn: 压力故障门控 (VPOWER 正常才判压力故障) */
#include "wheel_speed.h"     /* WheelSpeed_VehicleRefRpm: 间隙诊断速度门控 */
#include "cy_project.h"
#include "cy_device_headers.h"

/* ========================================================================== */
/*  内部状态                                                                   */
/* ========================================================================== */

/* ---- 轮速诊断: 100 点环形缓冲 ---- */
static uint16_t s_adc_buf[ADC_CH_WSS_NUM][ADC_DIAG_BUF_SIZE] = {{0}};
static uint8_t  s_adc_buf_idx        = 0u;
static uint8_t  s_adc_buf_fill_count = 0u;

/* ---- 压力 (前桥 CH6 / 后桥 CH7) ---- */
static uint16_t s_raw_front = 0u, s_raw_rear = 0u;
static uint16_t s_kpa_front = 0u, s_kpa_rear = 0u;
static bool     s_front_fault = false, s_rear_fault = false;
static uint8_t  s_front_err_cnt = 0u, s_front_ok_cnt = 0u;
static uint8_t  s_rear_err_cnt  = 0u, s_rear_ok_cnt  = 0u;

/* ---- VPOWER (CH8) ---- */
static uint16_t s_raw_vpwr = 0u, s_mv_vpwr = 0u, s_vpwr_x10 = 0u;
static uint8_t  s_vpwr_high_cnt = 0u, s_vpwr_high_ok_cnt = 0u;
static uint8_t  s_vpwr_low_cnt  = 0u, s_vpwr_low_ok_cnt  = 0u;
static bool     s_vpwr_high_alarm = false, s_vpwr_low_alarm = false;

/* ---- 轮速诊断结果 (对外) ---- */
volatile sensor_ch_diag_t      g_sensor_diag[ADC_CH_WSS_NUM] = {0};
volatile sensor_fault_status_t g_sensor_fault_status = {0};

/* ---- 组转换超时计数 ---- */
static uint16_t s_adc_timeout_cnt = 0u;

/* ---- P12.1 (独立 SAR1) ---- */
#if (HW_REV_P12_1_ADC == 1u)
static uint16_t s_p12_1_raw = 0u;
static uint16_t s_p12_1_mv  = 0u;
#endif

/* ========================================================================== */
/*  引脚 / 通道表 (按 CH0~CH8 顺序)                                            */
/* ========================================================================== */

typedef struct
{
    volatile stc_GPIO_PRT_t*  port;   /* 与 Cy_GPIO_Pin_Init 首参一致 */
    uint32_t                  pin;
    en_hsiom_sel_t            mux;    /* Px_y_GPIO 等 HSIOM 选择 */
    cy_en_adc_pin_address_t   an;     /* CY_ADC_PIN_ADDRESS_ANx */
} adc_ch_map_t;

static const adc_ch_map_t s_adc_ch_map[ADC_CH_TOTAL] =
{
    { ADC_DIAG_CH0_PORT, ADC_DIAG_CH0_PIN, ADC_DIAG_CH0_MUX, ADC_DIAG_CH0_AN },  /* 轮速 FL */
    { ADC_DIAG_CH1_PORT, ADC_DIAG_CH1_PIN, ADC_DIAG_CH1_MUX, ADC_DIAG_CH1_AN },  /* 轮速 RL */
    { ADC_DIAG_CH2_PORT, ADC_DIAG_CH2_PIN, ADC_DIAG_CH2_MUX, ADC_DIAG_CH2_AN },  /* 轮速 RR */
    { ADC_DIAG_CH3_PORT, ADC_DIAG_CH3_PIN, ADC_DIAG_CH3_MUX, ADC_DIAG_CH3_AN },  /* 轮速 FR */
    { ADC_DIAG_CH4_PORT, ADC_DIAG_CH4_PIN, ADC_DIAG_CH4_MUX, ADC_DIAG_CH4_AN },  /* 轮速 XL */
    { ADC_DIAG_CH5_PORT, ADC_DIAG_CH5_PIN, ADC_DIAG_CH5_MUX, ADC_DIAG_CH5_AN },  /* 轮速 XR */
    { ADC_CH_FRONT_PORT, ADC_CH_FRONT_PIN, ADC_CH_FRONT_MUX, ADC_CH_FRONT_AN },  /* 前桥压力 */
    { ADC_CH_REAR_PORT,  ADC_CH_REAR_PIN,  ADC_CH_REAR_MUX,  ADC_CH_REAR_AN  },  /* 后桥压力 */
    { ADC_CH_VPWR_PORT,  ADC_CH_VPWR_PIN,  ADC_CH_VPWR_MUX,  ADC_CH_VPWR_AN  },  /* VPOWER (组尾) */
};

/* ========================================================================== */
/*  SAR0 初始化 — 时钟 + 9 通道一个 Group                                      */
/* ========================================================================== */
static void sar0_init(void)
{
    /* ---- 时钟: 动态读取 clk_peri, 分频到 ≤26.67MHz ---- */
    uint32_t periFreq = 0u;
    Cy_SysClk_GetClkPeriFrequency(&periFreq);
    uint32_t divNum = (periFreq + ADC_SAR0_MAX_FREQ_HZ / 2u) / ADC_SAR0_MAX_FREQ_HZ;

    Cy_SysClk_PeriphAssignDivider(ADC_SAR0_PCLK, CY_SYSCLK_DIV_16_BIT, ADC_SAR0_CLK_DIV);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, ADC_SAR0_CLK_DIV, (divNum - 1ul));
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, ADC_SAR0_CLK_DIV);

    /* ---- 采样时间: ceil(412ns × f_adc) × 5 ---- */
    uint32_t actualAdcFreq = periFreq / divNum;
    uint32_t samplingCycle = (412ull * actualAdcFreq + 500000000ull) / 1000000000ull;
    if (samplingCycle < 2u) samplingCycle = 2u;
    samplingCycle *= 5u;

    /* ---- ADC 全局初始化: 整个工程只在这里调一次 Cy_Adc_Init(SAR0) ---- */
    cy_stc_adc_config_t adcConfig =
    {
        .preconditionTime = 0u, .powerupTime = 0u, .enableIdlePowerDown = false,
        .msbStretchMode = CY_ADC_MSB_STRETCH_MODE_2CYCLE,
        .enableHalfLsbConv = 1u,
        .sarMuxEnable = true, .adcEnable = true, .sarIpEnable = true,
    };
    Cy_Adc_Init(ADC_SAR0_MACRO, &adcConfig);

    /* ---- 引脚: 9 路 SARMUX 直连 (HSIOM=0 / ANALOG) ---- */
    {
        cy_stc_gpio_pin_config_t adcPinCfg =
        {
            .outVal = 0ul, .driveMode = CY_GPIO_DM_ANALOG, .hsiom = ADC_DIAG_CH0_MUX,
            .intEdge = 0ul, .intMask = 0ul, .vtrip = 0ul, .slewRate = 0ul, .driveSel = 0ul,
        };
        for (uint8_t ch = 0u; ch < ADC_CH_TOTAL; ch++)
        {
            adcPinCfg.hsiom = s_adc_ch_map[ch].mux;
            Cy_GPIO_Pin_Init(s_adc_ch_map[ch].port, s_adc_ch_map[ch].pin, &adcPinCfg);
        }
    }

    /* ---- 通道: CH0~CH8 合成一个 Group 自动序列, 组尾统一 CH8 ---- */
    {
        const cy_stc_adc_channel_config_t adcChBase =
        {
            .triggerSelection = CY_ADC_TRIGGER_OFF,
            .channelPriority = 0u,
            .preenptionType = CY_ADC_PREEMPTION_FINISH_RESUME,
            .isGroupEnd = false,
            .doneLevel = CY_ADC_DONE_LEVEL_PULSE,
            .extMuxSelect = 0u, .extMuxEnable = true,
            .preconditionMode = CY_ADC_PRECONDITION_MODE_OFF,
            .overlapDiagMode = CY_ADC_OVERLAP_DIAG_MODE_OFF,
            .sampleTime = samplingCycle,
            .calibrationValueSelect = CY_ADC_CALIBRATION_VALUE_REGULAR,
            .postProcessingMode = CY_ADC_POST_PROCESSING_MODE_NONE,
            .resultAlignment = CY_ADC_RESULT_ALIGNMENT_RIGHT,
            .signExtention = CY_ADC_SIGN_EXTENTION_UNSIGNED,
            .averageCount = 0u, .rightShift = 0u,
            .rangeDetectionMode = CY_ADC_RANGE_DETECTION_MODE_INSIDE_RANGE,
            .rangeDetectionLoThreshold = 0x0000u, .rangeDetectionHiThreshold = 0x0FFFu,
            .mask.grpDone = false, .mask.grpCancelled = false, .mask.grpOverflow = false,
            .mask.chRange = false, .mask.chPulse = false, .mask.chOverflow = false,
        };

        for (uint8_t ch = 0u; ch < ADC_CH_TOTAL; ch++)
        {
            cy_stc_adc_channel_config_t chCfg = adcChBase;
            chCfg.pinAddress  = s_adc_ch_map[ch].an;
            chCfg.portAddress = CY_ADC_PORT_ADDRESS_SARMUX0;

            if (ch == ADC_CH_VPWR)
            {
                /* 组尾: 产生 grpDone, 由它判断整组转换完成 */
                chCfg.isGroupEnd   = true;
                chCfg.mask.grpDone = true;
            }
            Cy_Adc_Channel_Init(&ADC_SAR0_MACRO->CH[ch], &chCfg);
        }
    }

    /* ---- 使能 9 个通道 ---- */
    for (uint8_t ch = 0u; ch < ADC_CH_TOTAL; ch++)
    {
        Cy_Adc_Channel_Enable(&ADC_SAR0_MACRO->CH[ch]);
    }
}

/* ========================================================================== */
/*  P12.1 初始化 (独立 SAR1 实例, HW_REV_P12_1_ADC 开关)                       */
/* ========================================================================== */
static void p12_1_init(void)
{
#if (HW_REV_P12_1_ADC == 1u)
    uint32_t periFreq = 0u;
    Cy_SysClk_GetClkPeriFrequency(&periFreq);
    uint32_t divNum = (periFreq + ADC_SAR0_MAX_FREQ_HZ / 2u) / ADC_SAR0_MAX_FREQ_HZ;

    Cy_SysClk_PeriphAssignDivider(P12_1_ADC_PCLK, CY_SYSCLK_DIV_16_BIT, P12_1_ADC_CLK_DIV);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, P12_1_ADC_CLK_DIV, (divNum - 1ul));
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, P12_1_ADC_CLK_DIV);

    uint32_t actualAdcFreq = periFreq / divNum;
    uint32_t samplingCycle = (412ull * actualAdcFreq + 500000000ull) / 1000000000ull;
    if (samplingCycle < 2u) samplingCycle = 2u;
    samplingCycle *= 5u;

    /* 引脚: P12.1 模拟直连 */
    {
        cy_stc_gpio_pin_config_t adcPinCfg =
        {
            .outVal = 0ul, .driveMode = CY_GPIO_DM_ANALOG, .hsiom = P12_1_ADC_MUX,
            .intEdge = 0ul, .intMask = 0ul, .vtrip = 0ul, .slewRate = 0ul, .driveSel = 0ul,
        };
        Cy_GPIO_Pin_Init(P12_1_ADC_PORT, P12_1_ADC_PIN, &adcPinCfg);
    }

    /* ADC 实例初始化 (独立 SAR1, 不影响 SAR0) */
    {
        cy_stc_adc_config_t adcConfig =
        {
            .preconditionTime = 0u, .powerupTime = 0u, .enableIdlePowerDown = false,
            .msbStretchMode = CY_ADC_MSB_STRETCH_MODE_2CYCLE,
            .enableHalfLsbConv = 1u,
            .sarMuxEnable = true, .adcEnable = true, .sarIpEnable = true,
        };
        Cy_Adc_Init(P12_1_ADC_MACRO, &adcConfig);
    }

    /* 单通道 CH0: AN5, 组头=组尾 */
    {
        cy_stc_adc_channel_config_t chCfg =
        {
            .triggerSelection       = CY_ADC_TRIGGER_OFF,
            .channelPriority        = 0u,
            .preenptionType         = CY_ADC_PREEMPTION_FINISH_RESUME,
            .isGroupEnd             = true,
            .doneLevel              = CY_ADC_DONE_LEVEL_PULSE,
            .pinAddress             = CY_ADC_PIN_ADDRESS_AN5,
            .portAddress            = CY_ADC_PORT_ADDRESS_SARMUX0,
            .extMuxSelect           = 0u, .extMuxEnable = true,
            .preconditionMode       = CY_ADC_PRECONDITION_MODE_OFF,
            .overlapDiagMode        = CY_ADC_OVERLAP_DIAG_MODE_OFF,
            .sampleTime             = samplingCycle,
            .calibrationValueSelect = CY_ADC_CALIBRATION_VALUE_REGULAR,
            .postProcessingMode     = CY_ADC_POST_PROCESSING_MODE_NONE,
            .resultAlignment        = CY_ADC_RESULT_ALIGNMENT_RIGHT,
            .signExtention          = CY_ADC_SIGN_EXTENTION_UNSIGNED,
            .averageCount           = 0u, .rightShift = 0u,
            .rangeDetectionMode     = CY_ADC_RANGE_DETECTION_MODE_INSIDE_RANGE,
            .rangeDetectionLoThreshold = 0x0000u, .rangeDetectionHiThreshold = 0x0FFFu,
            .mask.grpDone      = true,  .mask.grpCancelled = false, .mask.grpOverflow = false,
            .mask.chRange      = false, .mask.chPulse     = false, .mask.chOverflow  = false,
        };
        Cy_Adc_Channel_Init(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH], &chCfg);
    }

    Cy_Adc_Channel_Enable(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH]);
#endif
}

/* ========================================================================== */
/*  SAR0 组转换: 触发 + 等组尾 (带超时)                                         */
/* ========================================================================== */
static bool sar0_trigger_and_wait(void)
{
    Cy_Adc_Channel_SoftwareTrigger(&ADC_SAR0_MACRO->CH[ADC_GROUP_HEAD_CH]);

    cy_stc_adc_interrupt_source_t intr = { false };
    uint32_t guard = ADC_POLL_TIMEOUT_LOOP;

    do
    {
        intr = (cy_stc_adc_interrupt_source_t){ false };
        Cy_Adc_Channel_GetInterruptMaskedStatus(&ADC_SAR0_MACRO->CH[ADC_CH_VPWR], &intr);
    } while ((!intr.grpDone) && (--guard != 0ul));

    if (!intr.grpDone)
    {
        s_adc_timeout_cnt++;     /* SAR 异常: 本轮作废, 保留上一轮数据 */
        return false;
    }
    return true;
}

/* ========================================================================== */
/*  CH0~CH5 → 轮速诊断环形缓冲                                                 */
/* ========================================================================== */
static void wss_read_to_buffer(void)
{
    const uint8_t idx = s_adc_buf_idx;
    cy_stc_adc_interrupt_source_t intr = { false };

    for (uint8_t ch = 0u; ch < ADC_CH_WSS_NUM; ch++)
    {
        if (!IS_ADC_DIAG_CH_ENABLED(ch)) continue;

        cy_stc_adc_ch_status_t adc_status;
        uint16_t raw = 0u;
        if (Cy_Adc_Channel_GetResult(&ADC_SAR0_MACRO->CH[ch], &raw, &adc_status) == CY_ADC_SUCCESS
            && adc_status.valid)
        {
            s_adc_buf[ch][idx] = raw;
        }
        Cy_Adc_Channel_ClearInterruptStatus(&ADC_SAR0_MACRO->CH[ch], &intr);
    }

    s_adc_buf_idx = (uint8_t)((idx + 1u) % ADC_DIAG_BUF_SIZE);
    if (s_adc_buf_fill_count < ADC_DIAG_BUF_SIZE)
    {
        s_adc_buf_fill_count++;
    }
}

/* ========================================================================== */
/*  CH6~CH8 → 压力换算/故障消抖 + VPOWER 换算/报警消抖                          */
/* ========================================================================== */
static void press_vpwr_read(void)
{
    /* ---- 读取三通道结果 ---- */
    static const uint8_t s_ch_idx[3] = { ADC_CH_PRESS_FRONT, ADC_CH_PRESS_REAR, ADC_CH_VPWR };
    uint16_t raw[3]   = { 0u, 0u, 0u };
    bool     valid[3] = { false, false, false };
    cy_stc_adc_interrupt_source_t intr = { false };

    for (uint8_t i = 0u; i < 3u; i++)
    {
        const uint8_t hwCh = s_ch_idx[i];
        cy_stc_adc_ch_status_t adc_status;
        uint16_t r = 0u;
        if (Cy_Adc_Channel_GetResult(&ADC_SAR0_MACRO->CH[hwCh], &r, &adc_status) == CY_ADC_SUCCESS
            && adc_status.valid)
        {
            raw[i]   = r;
            valid[i] = true;
        }
        Cy_Adc_Channel_ClearInterruptStatus(&ADC_SAR0_MACRO->CH[hwCh], &intr);
    }

    /* ---- 保存原始值 (raw[0]=前桥, raw[1]=后桥, raw[2]=VPOWER) ---- */
    if (valid[0]) s_raw_front = raw[0];
    if (valid[1]) s_raw_rear  = raw[1];
    if (valid[2]) s_raw_vpwr  = raw[2];

    /* ---- VPOWER 电压换算 (mV) ---- */
    s_mv_vpwr = (uint16_t)((uint32_t)s_raw_vpwr * ADC_VREF_MV / ADC_12BIT_MAX);

    /* ---- 前桥压力故障诊断 (双层窗口 + 50次对称消抖, VPOWER 异常时跳过) ---- */
    /* VPOWER 正常窗口 18.0~32.0V (fact 0.1V): 之外的读值不可信, 不做压力判决 */
    const bool vpwr_ok = (PSWvIgn >= 180u && PSWvIgn <= 320u);
    if (vpwr_ok && (s_raw_front < PRESS_ERR_MIN_RAW || s_raw_front > PRESS_ERR_MAX_RAW))
    {
        s_front_ok_cnt = 0u;
        s_front_err_cnt++;
        if (s_front_err_cnt >= PRESS_FAULT_STREAK)
        {
            s_front_fault = true;
            s_front_err_cnt = PRESS_FAULT_STREAK;
        }
    }
    else
    {
        s_front_err_cnt = 0u;
        s_front_ok_cnt++;
        if (s_front_ok_cnt >= PRESS_FAULT_STREAK)
        {
            s_front_fault = false;
            s_front_ok_cnt = PRESS_FAULT_STREAK;
        }
    }

    /* ---- 后桥压力故障诊断 ---- */
    if (vpwr_ok && (s_raw_rear < PRESS_ERR_MIN_RAW || s_raw_rear > PRESS_ERR_MAX_RAW))
    {
        s_rear_ok_cnt = 0u;
        s_rear_err_cnt++;
        if (s_rear_err_cnt >= PRESS_FAULT_STREAK)
        {
            s_rear_fault = true;
            s_rear_err_cnt = PRESS_FAULT_STREAK;
        }
    }
    else
    {
        s_rear_err_cnt = 0u;
        s_rear_ok_cnt++;
        if (s_rear_ok_cnt >= PRESS_FAULT_STREAK)
        {
            s_rear_fault = false;
            s_rear_ok_cnt = PRESS_FAULT_STREAK;
        }
    }

    /* ---- 前桥压力换算 (0.5V~4.7V 线性, 容忍区 clamp) ---- */
    if (!s_front_fault)
    {
        if (s_raw_front >= PRESS_NORMAL_MIN_RAW && s_raw_front <= PRESS_NORMAL_MAX_RAW)
        {
            uint32_t kpa = (uint32_t)(s_raw_front - PRESS_NORMAL_MIN_RAW) * PRESS_MAX_KPA
                         / (PRESS_NORMAL_MAX_RAW - PRESS_NORMAL_MIN_RAW);
            s_kpa_front = (uint16_t)kpa;
        }
        else if (s_raw_front < PRESS_NORMAL_MIN_RAW)
        {
            s_kpa_front = 0u;
        }
        else
        {
            s_kpa_front = PRESS_MAX_KPA;
        }
    }
    else
    {
        s_kpa_front = 0u;
    }

    /* ---- 后桥压力换算 ---- */
    if (!s_rear_fault)
    {
        if (s_raw_rear >= PRESS_NORMAL_MIN_RAW && s_raw_rear <= PRESS_NORMAL_MAX_RAW)
        {
            uint32_t kpa = (uint32_t)(s_raw_rear - PRESS_NORMAL_MIN_RAW) * PRESS_MAX_KPA
                         / (PRESS_NORMAL_MAX_RAW - PRESS_NORMAL_MIN_RAW);
            s_kpa_rear = (uint16_t)kpa;
        }
        else if (s_raw_rear < PRESS_NORMAL_MIN_RAW)
        {
            s_kpa_rear = 0u;
        }
        else
        {
            s_kpa_rear = PRESS_MAX_KPA;
        }
    }
    else
    {
        s_kpa_rear = 0u;
    }

    /* ---- VPOWER 电压换算: ADC_V(mV) × 分压比 → 实际电压 (×10, fact 0.1V) ---- */
    {
        uint32_t vpwr = (uint32_t)s_mv_vpwr * VPWR_DIVIDER_RATIO_NUM / VPWR_DIVIDER_RATIO_DEN;
        s_vpwr_x10 = (uint16_t)(vpwr / 100u);
    }

    /* ---- VPOWER 高压报警 (对称消抖) ---- */
    if (s_vpwr_x10 >= VPWR_ALARM_HIGH_MV)
    {
        s_vpwr_high_ok_cnt = 0u;
        s_vpwr_high_cnt++;
        if (s_vpwr_high_cnt >= VPWR_ALARM_STREAK)
        {
            s_vpwr_high_alarm = true;
            s_vpwr_high_cnt = VPWR_ALARM_STREAK;
        }
    }
    else
    {
        s_vpwr_high_cnt = 0u;
        s_vpwr_high_ok_cnt++;
        if (s_vpwr_high_ok_cnt >= VPWR_ALARM_STREAK)
        {
            s_vpwr_high_alarm = false;
            s_vpwr_high_ok_cnt = VPWR_ALARM_STREAK;
        }
    }

    /* ---- VPOWER 低压报警 (对称消抖) ---- */
    if (s_vpwr_x10 <= VPWR_ALARM_LOW_MV && s_vpwr_x10 > 0u)
    {
        s_vpwr_low_ok_cnt = 0u;
        s_vpwr_low_cnt++;
        if (s_vpwr_low_cnt >= VPWR_ALARM_STREAK)
        {
            s_vpwr_low_alarm = true;
            s_vpwr_low_cnt = VPWR_ALARM_STREAK;
        }
    }
    else
    {
        s_vpwr_low_cnt = 0u;
        s_vpwr_low_ok_cnt++;
        if (s_vpwr_low_ok_cnt >= VPWR_ALARM_STREAK)
        {
            s_vpwr_low_alarm = false;
            s_vpwr_low_ok_cnt = VPWR_ALARM_STREAK;
        }
    }
}

/* ========================================================================== */
/*  P12.1 单通道读取                                                           */
/* ========================================================================== */
static void p12_1_read(void)
{
#if (HW_REV_P12_1_ADC == 1u)
    cy_stc_adc_interrupt_source_t intr = { false };
    uint32_t guard = ADC_POLL_TIMEOUT_LOOP;

    Cy_Adc_Channel_SoftwareTrigger(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH]);

    do
    {
        intr = (cy_stc_adc_interrupt_source_t){ false };
        Cy_Adc_Channel_GetInterruptMaskedStatus(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH], &intr);
    } while ((!intr.grpDone) && (--guard != 0ul));

    if (intr.grpDone)
    {
        cy_stc_adc_ch_status_t adc_status;
        uint16_t r = 0u;
        if (Cy_Adc_Channel_GetResult(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH], &r, &adc_status) == CY_ADC_SUCCESS
            && adc_status.valid)
        {
            s_p12_1_raw = r;
            s_p12_1_mv  = (uint16_t)((uint32_t)r * P12_1_ADC_VREF_MV / P12_1_ADC_12BIT_MAX);
        }
        Cy_Adc_Channel_ClearInterruptStatus(&P12_1_ADC_MACRO->CH[P12_1_ADC_CH], &intr);
    }
    else
    {
        s_adc_timeout_cnt++;
    }
#endif
}

/* ========================================================================== */
/*  轮速传感器诊断判决 (滞回 + 速度门控)                                        */
/* ========================================================================== */
static void wss_diag_process(const uint16_t avg_raw[ADC_CH_WSS_NUM],
                             const uint16_t amplitude_raw[ADC_CH_WSS_NUM])
{
    const uint16_t vehicle_rpm = WheelSpeed_VehicleRefRpm();

    for (uint8_t i = 0u; i < ADC_CH_WSS_NUM; i++)
    {
        if (!IS_ADC_DIAG_CH_ENABLED(i)) continue;

        g_sensor_diag[i].adc_raw    = avg_raw[i];
        g_sensor_diag[i].voltage_mv = (avg_raw[i] * ADC_VREF_MV) / ADC_12BIT_MAX;

        /* 低速窗口 (车速低于门限): 判开路/短路 */
        if (vehicle_rpm < DIAG_GAP_RPM_THRESH)
        {
            /* 开路故障 (~0.55V~0.68V) — 滞回比较 */
            if (!g_sensor_diag[i].fault_open)
            {
                if (avg_raw[i] >= DIAG_OPEN_MIN && avg_raw[i] <= DIAG_OPEN_MAX)
                    g_sensor_diag[i].fault_open = true;
            }
            else
            {
                if (avg_raw[i] < (DIAG_OPEN_MIN - DIAG_OPEN_HYST) ||
                    avg_raw[i] > (DIAG_OPEN_MAX + DIAG_OPEN_HYST))
                    g_sensor_diag[i].fault_open = false;
            }

            /* 短路故障 (~3.0V~3.4V) — 滞回比较 */
            if (!g_sensor_diag[i].fault_short)
            {
                if (avg_raw[i] >= DIAG_SHORT_MIN && avg_raw[i] <= DIAG_SHORT_MAX)
                    g_sensor_diag[i].fault_short = true;
            }
            else
            {
                if (avg_raw[i] < (DIAG_SHORT_MIN - DIAG_SHORT_HYST) ||
                    avg_raw[i] > (DIAG_SHORT_MAX + DIAG_SHORT_HYST))
                    g_sensor_diag[i].fault_short = false;
            }
        }

        /* 高速窗口 (车速高于门限): 判间隙过大 (峰峰值衰减) */
        if (vehicle_rpm > DIAG_GAP_RPM_THRESH)
        {
            if (!g_sensor_diag[i].fault_gap_too_large)
            {
                if (amplitude_raw[i] < DIAG_GAP_AMPL_MIN)
                    g_sensor_diag[i].fault_gap_too_large = true;
            }
            else
            {
                if (amplitude_raw[i] >= (DIAG_GAP_AMPL_MIN + DIAG_GAP_AMPL_HYST))
                    g_sensor_diag[i].fault_gap_too_large = false;
            }
        }
    }

    /* ---- 更新故障汇总结构体 ---- */
    g_sensor_fault_status.ch0_fault_open          = (IS_ADC_DIAG_CH_ENABLED(0) && g_sensor_diag[0].fault_open);
    g_sensor_fault_status.ch0_fault_short         = (IS_ADC_DIAG_CH_ENABLED(0) && g_sensor_diag[0].fault_short);
    g_sensor_fault_status.ch0_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(0) && g_sensor_diag[0].fault_gap_too_large);

    g_sensor_fault_status.ch1_fault_open          = (IS_ADC_DIAG_CH_ENABLED(1) && g_sensor_diag[1].fault_open);
    g_sensor_fault_status.ch1_fault_short         = (IS_ADC_DIAG_CH_ENABLED(1) && g_sensor_diag[1].fault_short);
    g_sensor_fault_status.ch1_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(1) && g_sensor_diag[1].fault_gap_too_large);

    g_sensor_fault_status.ch2_fault_open          = (IS_ADC_DIAG_CH_ENABLED(2) && g_sensor_diag[2].fault_open);
    g_sensor_fault_status.ch2_fault_short         = (IS_ADC_DIAG_CH_ENABLED(2) && g_sensor_diag[2].fault_short);
    g_sensor_fault_status.ch2_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(2) && g_sensor_diag[2].fault_gap_too_large);

    g_sensor_fault_status.ch3_fault_open          = (IS_ADC_DIAG_CH_ENABLED(3) && g_sensor_diag[3].fault_open);
    g_sensor_fault_status.ch3_fault_short         = (IS_ADC_DIAG_CH_ENABLED(3) && g_sensor_diag[3].fault_short);
    g_sensor_fault_status.ch3_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(3) && g_sensor_diag[3].fault_gap_too_large);

    g_sensor_fault_status.ch4_fault_open          = (IS_ADC_DIAG_CH_ENABLED(4) && g_sensor_diag[4].fault_open);
    g_sensor_fault_status.ch4_fault_short         = (IS_ADC_DIAG_CH_ENABLED(4) && g_sensor_diag[4].fault_short);
    g_sensor_fault_status.ch4_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(4) && g_sensor_diag[4].fault_gap_too_large);

    g_sensor_fault_status.ch5_fault_open          = (IS_ADC_DIAG_CH_ENABLED(5) && g_sensor_diag[5].fault_open);
    g_sensor_fault_status.ch5_fault_short         = (IS_ADC_DIAG_CH_ENABLED(5) && g_sensor_diag[5].fault_short);
    g_sensor_fault_status.ch5_fault_gap_too_large = (IS_ADC_DIAG_CH_ENABLED(5) && g_sensor_diag[5].fault_gap_too_large);
}

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */
void Adc_Init(void)
{
    sar0_init();
    p12_1_init();
}

void Adc_Read(void)
{
    /* ① SAR0 组转换 (CH0~CH8), 超时则本轮整体作废 */
    if (sar0_trigger_and_wait())
    {
        /* ② 轮速诊断通道入缓冲  ③ 压力/VPOWER 换算与消抖 */
        wss_read_to_buffer();
        press_vpwr_read();
    }

    /* ④ P12.1 (独立 SAR1 实例) */
    p12_1_read();
}

void Adc_Process(void)
{
    /* 缓冲未满 100 点 (<1s) → 仅采样不判决 */
    if (s_adc_buf_fill_count < ADC_DIAG_BUF_SIZE)
    {
        return;
    }

    /* ---- 统计: 100 点平均值 / 峰峰值 ---- */
    uint16_t avg_raw[ADC_CH_WSS_NUM]       = {0};
    uint16_t amplitude_raw[ADC_CH_WSS_NUM] = {0};

    for (uint8_t ch = 0u; ch < ADC_CH_WSS_NUM; ch++)
    {
        if (!IS_ADC_DIAG_CH_ENABLED(ch)) continue;

        uint32_t sum  = 0u;
        uint16_t vmin = 0xFFFu;
        uint16_t vmax = 0u;

        for (uint8_t k = 0u; k < ADC_DIAG_BUF_SIZE; k++)
        {
            const uint16_t v = s_adc_buf[ch][k];
            sum += v;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }

        avg_raw[ch]       = (uint16_t)(sum / ADC_DIAG_BUF_SIZE);
        amplitude_raw[ch] = (uint16_t)(vmax - vmin);
    }

    /* ---- 判决 (滞回 + 速度门控) ---- */
    wss_diag_process(avg_raw, amplitude_raw);
}

uint16_t Adc_GetTimeoutCount(void)
{
    return s_adc_timeout_cnt;
}

/* ---- 压力查询 ---- */
uint16_t Pressure_GetFrontRaw(void)  { return s_raw_front; }
uint16_t Pressure_GetRearRaw(void)   { return s_raw_rear; }
uint16_t Pressure_GetFrontKpa(void)  { return s_kpa_front; }
uint16_t Pressure_GetRearKpa(void)   { return s_kpa_rear; }
bool     Pressure_IsFrontFault(void) { return s_front_fault; }
bool     Pressure_IsRearFault(void)  { return s_rear_fault; }

/* ---- VPOWER 查询 ---- */
uint16_t Vpower_GetRaw(void)         { return s_raw_vpwr; }
uint16_t Vpower_GetVoltage(void)     { return s_vpwr_x10; }
bool     Vpower_IsHighAlarm(void)    { return s_vpwr_high_alarm; }
bool     Vpower_IsLowAlarm(void)     { return s_vpwr_low_alarm; }

/* ---- P12.1 查询 ---- */
uint16_t P12_1_GetRaw(void)
{
#if (HW_REV_P12_1_ADC == 1u)
    return s_p12_1_raw;
#else
    return 0u;
#endif
}

uint16_t P12_1_GetVoltageMv(void)
{
#if (HW_REV_P12_1_ADC == 1u)
    return s_p12_1_mv;
#else
    return 0u;
#endif
}
