/**
 * @file    valve_hold_test.h
 * @brief   21口 / 22口 阶梯进气保压标定测试 (VHT_CHANNEL 选通道)
 *
 * @details 目的: 标定"多大进气脉宽 → 每步升多少 kPa"(阶梯增压标定), 而非冲 8atm。
 *
 * @details 硬件前提 (2026-09-08 确认):
 *    - 21口腔压由 PSWBrkPreX2_7 读取; 22口腔压由 PSWBrkPreX3_10 读取 (kPa)
 *    - 进气阀 = 常闭 NC, 通电才进气
 *    - 出气阀 = 常开 NO, **必须持续通电才能关住**, 断电即排空
 *    - 气源 8atm ≈ 800kPa (传感器量程 0~1050kPa)
 *
 * @details 阀映射 (VHT_CHANNEL 选通道, VHT_USE_FRONT_VALVES 选原阀/前桥阀):
 *    0 = 21口 → VALVE_21IN / VALVE_21OUT (U12, X3_7 / X3_10)  压力 PSWBrkPreX2_7  低边 P2.0
 *    1 = 22口 → VALVE_22IN / VALVE_22OUT (U13, X4_16 / X2_16) 压力 PSWBrkPreX3_10 低边 P2.0 + **P6.2**
 *    VHT_USE_FRONT_VALVES=1 时可临时用前桥阀代替 (21口→LFI/LFO, 22口→RFI/RFO)。
 *    ⚠ 22口 排气阀 (VALVE_22OUT) 低边是 P6.2, 已由 vht_lowside_enable() 随通道自动打开;
 *      若漏开则排气阀无法通电关闭 → 完全保不住压。
 *
 * @details 时序 (每步 = 1 个 PWM 周期):
 * @code
 *     进气阀 (21IN/22IN):  |<-- 进气 PULSE -->|<-------- 保压 HOLD -------->|
 *                          (前段高, cc0=0)    (低电平, 阀关闭)
 *     排气阀 (21OUT/22OUT): 全程满占空恒高 (保持关闭), 直到排空阶段才断电
 * @endcode
 *
 *   脉宽由 TCPWM 硬件产生, 软件只做"步计数 + 采样", 不参与脉宽计时
 *   (主循环 10ms 节拍, 软件无法精确开关 ms 级脉宽)。
 *
 * @note 周期限制: valve_pwm 时钟 100kHz (1 步 = 10μs), period 为 16bit 计数 → 硬上限
 *       65535 步 = 655.35ms (原 500ms 只是 ABS 控阀路径的推荐值)。
 *       本模块静态检查取 ≤ 600ms, 标定用的 520ms 已放开;
 *       周期还必须是 10ms 整数倍才能与 10ms 任务对齐。
 */

#ifndef VALVE_HOLD_TEST_H
#define VALVE_HOLD_TEST_H

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/*  可配置参数 (标定期间现场改数重烧)                                          */
/* ========================================================================== */

/* 充气模式:
 *   0 = 阶梯脉冲 (每周期 开 PULSE + 关 HOLD, 逐步采样)                        ← 当前
 *   1 = 连续全开 (阀一直开, 不关阀, 最长 VHT_CONT_MAX_MS; 到 P_FULL 提前停) */
#define VHT_CONTINUOUS_MODE     0u
#define VHT_CONT_MAX_MS         6000u   /* 连续充气时长上限 (ms); 到点或压力达 P_FULL 即停, 转 FINAL 长保压 */

#define VHT_IN_PULSE_MS         40u     /* 单次进气脉宽 (ms) —— **进气阀占空比调节处**
                                         * 占空比 = 脉宽 / (脉宽+保压) = 10 / 460 ≈ 2.2%
                                         * (必须是 10ms 整数倍, 采样相位依赖此值) */
#define VHT_HOLD_MS             460u    /* 单步保压时长 (ms) — 与脉宽合计须为 10ms 整数倍
                                         * (当前周期 = 10+450 = 460ms; 20 步 ≈9.2s
                                         *  保压越长越容易看出泄漏) */
#define VHT_STEP_NUM            200u     /* 阶梯次数上限 === 标定用:
                                         *   本阶段目的不是冲到 780kPa, 而是标定"多大脉宽 → 每步升多少 kPa",
                                         *   20 步足够看出 5kPa / 10kPa 的梯度 (此值下 P_FULL 不会提前触发结束)。
                                         *   看每步实际升压: g_vht.dp_step[] (或 CAN 0x721 的 VHT_StepRise) */
#define VHT_P_FULL_KPA          780u    /* 充满阈值 (kPa) */
#define VHT_P_MIN_RISE_KPA      5u      /* 最小有效升压 (kPa), 低于此判 NO_RISE */
#define VHT_P_TOL_KPA_100MS     20u     /* 保压容差: kPa / 100ms */
#define VHT_FINAL_HOLD_MS       1000u   /* 末次长保压观察时长 (ms) */
#define VHT_P_VENT_DONE_KPA     50u     /* 排空完成阈值 (kPa) */
#define VHT_VENT_TIMEOUT_MS     2000u   /* 排空超时 (ms) */
#define VHT_TOTAL_TIMEOUT_MS    60000u  /* 全流程硬超时保护 (ms) — 160×120ms + 保压 + 排空 */
#define VHT_PREP_WAIT_MS        5000u   /* 启动条件 (VPOWER/压力/阀故障) 等待上限 (ms):
                                         * 上电初期供电未稳时逐拍重试, 超过此值才判 COND_FAIL */

/* ========================================================================== */
/*  占空比调节 (现场自调, 不用改逻辑)                                            */
/*                                                                            */
/*  进气阀: 占空比 = VHT_IN_PULSE_MS / (VHT_IN_PULSE_MS + VHT_HOLD_MS)           */
/*          当前 10 / 460 ≈ 2.2%; 想要多少% 就改 VHT_IN_PULSE_MS (脉宽 ms),       */
/*          脉宽必须保持 10ms 整数倍 (采样拍的相位依赖它)。                        */
/*                                                                            */
/*  排气阀: 保压期必须**持续通电**才能把 NO 阀关住, 这里给的是"保压期通电占空比":   */
/*          100 = 直流恒高 (默认; 吸力最大、关得最牢)                            */
/*          <100 = 斩波保持 (周期 VHT_OUT_CHOP_PERIOD_MS, 高电平占 PCT%),          */
/*                 用于试验"降平均电流/降发热后阀还能不能关住"                     */
/*  ⚠ 排气阀占空比不是越小越好: 它是靠电流吸住阀门的, 降占空比 = 降吸力,            */
/*     若本来带压就关不住, 降占空比只会更关不住 (要降发热请先查驱动芯片限流)。        */
/* ========================================================================== */
#define VHT_OUT_HOLD_DUTY_PCT   100u    /* 排气阀保压期通电占空比 % (0~100) */
#define VHT_OUT_CHOP_PERIOD_MS  100u     /* 排气阀斩波周期 ms (≥10, ≤600; 仅 <100% 时生效) */

/* 阀极性逻辑 (2026-09-09 确认 21口/左前 LFI+LFO; 2026-09-10 用户确认 22口 亦相同):
 *   0 = 进气 NC (高电平打开) / 排气 NO (高电平关闭, 低电平排气)   ← 当前, 已确认
 *   1 = 进气 NO / 排气 NC   (备用, 目前无需切换)
 *   21口/22口 与普通 ABS 阀极性一致 (进气=后段高, 排气=前段高), 由 InValAct21/22、OutValAct21/22 驱动;
 *   本宏只决定 VHT 自己用 SetDuty 生成的"打开窗口"相位, 两种取值下采样相位一致。 */
#define VHT_VALVE_LOGIC         0u

/* 测试通道: 0 = 21口 (VALVE_21IN/21OUT, 压力源 PSWBrkPreX2_7, 低边 P2.0)          ← 当前
 *           1 = 22口 (VALVE_22IN/22OUT, 压力源 PSWBrkPreX3_10, 低边 P2.0 + P6.2) */
#define VHT_CHANNEL             0u

/* 阀映射: 线缆已于 2026-09-15 改接回 21口/22口原阀 (U12/U13)
 *   1 = 用前桥阀代替: 21口 → 左前进 LFI / 左前排 LFO ; 22口 → 右前进 RFI / 右前排 RFO
 *   0 = 用回原阀 (当前): 21口 → VALVE_21IN / VALVE_21OUT ; 22口 → VALVE_22IN / VALVE_22OUT
 *   各通道的压力源: 21口 = PSWBrkPreX2_7, 22口 = PSWBrkPreX3_10 (见 .c 顶部) */
#define VHT_USE_FRONT_VALVES    0u

/* 试验台接线确认: 21口 阀 (U12) 线圈地电路上不经 ASR 低边, 但按试验台要求仍开 P2.0,
 * 开着无副作用, 故保持 1 (前桥阀代替时同样适用)。 */
#define VHT_LOWSIDE_ENABLE      1u

/* 1 = 上电后自动跑一次 (标定用, 观察时置 1); 0 = 需外部调用 ValveHoldTest_Start() */
#define VHT_AUTO_START          0u

/* ========================================================================== */
/*  CAN 实时上报 (CAN1 调试口, 供上位机/CAN盒画实时曲线)                        */
/* ========================================================================== */
#define VHT_CAN_REPORT_ENABLE   1u

#define VHT_CAN_ID_TRACE        0x720u   /* 实时曲线帧 — 每 10ms 一帧 */
#define VHT_CAN_ID_STEP         0x721u   /* 每步结果帧 — 每步结束一帧 */
#define VHT_CAN_ID_RESULT       0x722u   /* 最终汇总帧 — 结束时一帧 */

/* ========================================================================== */
/*  派生参数与静态检查                                                         */
/* ========================================================================== */

/** 单步 PWM 周期 (ms) = 进气 + 保压 */
#define VHT_CYCLE_MS            (VHT_IN_PULSE_MS + VHT_HOLD_MS)

#if ((VHT_CYCLE_MS % 10u) != 0u)
#error "VHT: (VHT_IN_PULSE_MS + VHT_HOLD_MS) must be a multiple of 10ms"
#endif

#if (VHT_CYCLE_MS > 600u)
#error "VHT: PWM period limit is 600ms (TCPWM 16bit @100kHz -> hard cap 655.35ms)"
#endif

#if (VHT_OUT_HOLD_DUTY_PCT > 100u)
#error "VHT: VHT_OUT_HOLD_DUTY_PCT must be 0~100 (%)"
#endif

#if ((VHT_OUT_HOLD_DUTY_PCT < 100u) && ((VHT_OUT_CHOP_PERIOD_MS < 10u) || (VHT_OUT_CHOP_PERIOD_MS > 600u)))
#error "VHT: VHT_OUT_CHOP_PERIOD_MS must be 10~600ms (only used when VHT_OUT_HOLD_DUTY_PCT < 100)"
#endif

/** 单步 10ms 节拍数 */
#define VHT_TICK_PER_STEP       (VHT_CYCLE_MS / 10u)

/* 关阀后延迟多久采 P_a (避开关阀瞬态), 必须 < VHT_HOLD_MS - 10 */
#define VHT_SAMPLE_A_DELAY_MS   20u

/* 采样拍号 (步内, 0 起; tick=n ↔ 周期起点后 n*10ms):
 *   拍0      = PWM 周期起点 = 进气开始
 *   SAMPLE_A = 关阀后 VHT_SAMPLE_A_DELAY_MS (随脉宽自适应, 防止落进脉冲期)
 *   SAMPLE_B = 本步末 (下次脉冲前 10ms) */
#define VHT_TICK_SAMPLE_A       ((VHT_IN_PULSE_MS + VHT_SAMPLE_A_DELAY_MS) / 10u)
#define VHT_TICK_SAMPLE_B       (VHT_TICK_PER_STEP - 1u)

#if (VHT_TICK_SAMPLE_A >= VHT_TICK_SAMPLE_B)
#error "VHT: sample A falls into next pulse — increase VHT_HOLD_MS or shorten pulse"
#endif

/** 单次保压判定实际采样间隔 (ms) */
#define VHT_HOLD_SAMPLE_MS      ((VHT_TICK_SAMPLE_B - VHT_TICK_SAMPLE_A) * 10u)

/* ========================================================================== */
/*  状态与结果                                                                 */
/* ========================================================================== */

typedef enum
{
    VHT_IDLE = 0u,      /* 未运行 */
    VHT_PREP,           /* 准备: 条件检查 + 开低边 + 关出气阀 + 起进气脉冲 */
    VHT_STEP,           /* 阶梯进气 + 保压采样 (VHT_CONTINUOUS_MODE=0) */
    VHT_FINAL,          /* 末次长保压观察 */
    VHT_VENT,           /* 排空 (两阀断电) */
    VHT_CHARGE,         /* 连续全开充气 (VHT_CONTINUOUS_MODE=1) */
    VHT_DONE            /* 结束 */
} vht_state_t;

typedef enum
{
    VHT_RES_NONE = 0u,  /* 未出结果 */
    VHT_RES_OK,         /* 充满 + 保压合格 + 排空正常 */
    VHT_RES_NOT_FULL,   /* VHT_STEP_NUM 次后仍未达到 P_FULL */
    VHT_RES_LEAK,       /* 保压压变超差 */
    VHT_RES_NO_RISE,    /* 压力基本没上升 (阀未动作 / 气路不通 / 低边未开) */
    VHT_RES_VENT_FAIL,  /* 排空超时 */
    VHT_RES_COND_FAIL,  /* 启动条件不满足 (供电/压力传感器/阀故障) */
    VHT_RES_ABORT       /* 被中止或总超时 */
} vht_result_t;

typedef struct
{
    uint8_t  state;                     /* vht_state_t */
    uint8_t  step;                      /* 当前阶梯序号 (1..VHT_STEP_NUM) */
    uint8_t  steps_done;                /* 实际完成步数 */
    uint16_t tick;                      /* 步内 10ms 节拍 */
    uint32_t total_tick;                /* 总节拍 (超时保护) */

    uint16_t p_init;                    /* 起始压力 (kPa) */
    uint16_t p_a;                       /* 本步保压起点采样 */
    uint16_t p_b;                       /* 本步保压终点采样 */
    uint16_t p_step[VHT_STEP_NUM];      /* 每步压力 (kPa) */
    int16_t  d_hold[VHT_STEP_NUM];      /* 每步保压压变 (kPa, 带符号) */
    int16_t  dp_step[VHT_STEP_NUM];     /* 每步实际升压 ΔP (kPa, 带符号):
                                         *   第 1 步 = p_step[0] - p_init, 第 n 步 = p_step[n-1] - p_step[n-2]
                                         *   ← 标定"脉宽 ↔ 每步升压"直接看这个数组 */

    uint16_t p_final_start;             /* 末次长保压起点 */
    uint16_t p_final_end;               /* 末次长保压终点 (进入排空前保存, 用于算真实泄漏率) */
    uint16_t p_final;                   /* 当前压力 (排空阶段持续刷新, 用于观察排空过程) */

    uint8_t  full_flag;                 /* 1 = 达到充满阈值 */
    uint8_t  leak_flag;                 /* 1 = 保压压变超差 */
    uint8_t  result;                    /* vht_result_t */
} vht_data_t;

/** 测试结果 (Live Watch 直接观察) */
extern volatile vht_data_t g_vht;

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

/**
 * @brief   10ms 周期处理 (主循环调用, 必须晚于 PSWData_Refresh 且早于 ValveDiag_Process)。
 * @note    未启动时立即返回, 开销可忽略。
 */
void ValveHoldTest_Process(void);

/** @brief 启动测试 (重复调用无效, 运行中调用被忽略) */
void ValveHoldTest_Start(void);

/** @brief 中止测试 (立即断电排空, 恢复低边/诊断让路) */
void ValveHoldTest_Abort(void);

/** @return true = 正在运行 */
bool ValveHoldTest_IsRunning(void);

#endif /* VALVE_HOLD_TEST_H */
