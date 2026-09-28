/**
 * @file    bts724g.h
 * @brief   BTS724G 五片阀驱动器 (IN 控制 + ST 回读 + 短路检测)
 *
 * @details 本模块同时处理诊断: ValveDiag_Process() 每 10ms 调用一次, 40ms 执行 1 步，
 *          4 步轮转 (阀开路 → 阀短路 → 芯片开路 → 芯片短路), 每步测 1 个阀/芯片，
 *          17 阀 × 2 步 + 5 芯片 × 2 步 = 44 步 ≈ 1.76s 全覆盖。
 *
 * @details 硬件连接 (5 片 BTS724G, 控制 17 个 24V 电磁阀):
 *
 *   所有 INx 串联 2k/0603 电阻 (限流)
 *   所有 STx 串联 2k/0603 + 5.1k/0603 上拉到 VCC (5V)
 *   STx GPIO 配置: HIGHZ (禁止 PULLUP，分压器会将电平推入阈值灰区)
 *
 *   完整映射 (valve_id | 物理阀 | 芯片 | IN | IN脚 | TCPWM LINE | ST组 | ST脚 | ST伙伴,
 *             与 docs/阀映射表.md 一致, 电路图核对):
 *      0  后桥右 进 RRI   U6  IN1  P22.2   CNT32  LINE32    ST12  P5.2    1
 *      1  后桥右 排 RRO   U6  IN2  P22.1   CNT33  LINE33    ST12  P5.2    0
 *      2  后桥左 排 LRO   U6  IN3  P22.0   CNT34  LINE34    ST34  P5.1    3
 *      3  后桥左 进 LRI   U6  IN4  P21.5   CNT37  LINE37    ST34  P5.1    2
 *      4  前桥右 进 RFI   U9  IN1  P0.0    CNT18  LINE18    ST12  P6.0    5
 *      5  前桥右 排 RFO   U9  IN2  P23.4   CNT25  LINE25    ST12  P6.0    4
 *      6  前桥左 排 LFO   U9  IN3  P23.3   CNT267 LINE267   ST34  P5.3    7
 *      7  前桥左 进 LFI   U9  IN4  P22.3   CNT31  LINE31    ST34  P5.3    6
 *      8  21口 进  21IN   U12 IN1  P19.1   CNT26  LINE26    ST12  P2.3    9
 *      9  21口 排  21OUT  U12 IN2  P19.0   CNT259 LINE259   ST12  P2.3    8
 *     10  TR_ASR         U12 IN3  P18.6   CNT51  LINE51    ST34  P2.2    独占
 *     11  22口 进  22IN   U13 IN1  P18.3   CNT54  LINE54    ST12  P0.1    12
 *     12  22口 排  22OUT  U13 IN2  P18.5   CNT52  LINE52    ST12  P0.1    11
 *     13  辅助桥右 进 RXI U19 IN1  P21.1   CNT41  LINE41    ST12  P14.2   14
 *     14  辅助桥右 排 RXO U19 IN2  P21.0   CNT42  LINE42    ST12  P14.2   13
 *     15  辅助桥左 排 LXO U19 IN3  P19.3   CNT28  LINE28    ST34  P3.1    16
 *     16  辅助桥左 进 LXI U19 IN4  P19.2   CNT27  LINE27    ST34  P3.1    15
 *
 *   五片芯片汇总:
 *     U6  : 后桥右IN/OUT + 后桥左OUT/IN    ST12=P5.2,  ST34=P5.1   短路组 A (P11.1)
 *     U9  : 前桥右IN/OUT + 前桥左OUT/IN    ST12=P6.0,  ST34=P5.3   短路组 A (P11.1)
 *     U12 : 21口 进 / 21口 排 / TR_ASR     ST12=P2.3,  ST34=P2.2   短路组 B (P17.2)
 *     U13 : 22口 进 / 22口 排              ST12=P0.1,  ST34=无     短路组 B (P17.2)
 *     U19 : RXI/RXO/LXO/LXI                ST12=P14.2, ST34=P3.1   短路组 B (P17.2)
 *
 *   ST 组: 每 2 通道共享 1 个 ST 组 (开漏线与): IN1/IN2 → ST12, IN3/IN4 → ST34。
 *          测试某阀时其 ST 伙伴必须关闭, 否则 ST 电平被伙伴污染。
 *          TR_ASR (U12): IN3/IN4 在板上并接到同一 MCU 引脚 P18.6, 且芯片 OUT3/OUT4 也并联
 *          驱动同一线圈 (双通道分流) → 一路 PWM 即同时驱动两通道, 故本模块只配 1 路
 *          (CNT51 LINE51); ST34 只此一线圈, 独占无伙伴 (2026-09-10 用户确认硬件接法)。
 *          U13 无 ST34。
 *
 *   外部短路检测电路 (汇聚型, 2 个 GPIO):
 *     A 组: U6(4) + U9(4) = 8 阀 → FAR_CK2 (P11.1)
 *     B 组: U19(4) + U12(3) + U13(2) = 9 阀 → FAR_CK (P17.2)
 *
 *   (阀名对应的物理桥位见下方 valve_id_t 枚举注释, 不在此重复)
 */

#ifndef BTS724G_H
#define BTS724G_H

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/*  常量定义                                                                   */
/* ========================================================================== */
#define VALVE_NUM_TOTAL             17u     /* 电磁阀总数 */
#define BTS724G_NUM_CHIPS           5u      /* BTS724G 芯片数 */
#define BTS724G_NUM_CH_PER_CHIP     4u      /* 每片最大通道数 */

/* ========================================================================== */
/*  阀名枚举                                                                   */
/*                                                                            */
/*  命名规则:                                                                  */
/*    R/L  = 右/左                                                             */
/*    F    = 前桥                                                              */
/*    X    = 辅助桥                                                            */
/*    第二个R = 后桥                                                            */
/*    I/O  = 进气/排气                                                         */
/* ========================================================================== */

typedef enum
{
    /* 芯片 1 (U6): 后桥阀 */
    VALVE_RRI    = 0u,   /* 后桥右进气阀 */
    VALVE_RRO    = 1u,   /* 后桥右排气阀 */
    VALVE_LRO    = 2u,   /* 后桥左排气阀 */
    VALVE_LRI    = 3u,   /* 后桥左进气阀 */
    /* 芯片 2 (U9): 前桥阀 */
    VALVE_RFI    = 4u,   /* 前桥右进气阀 */
    VALVE_RFO    = 5u,   /* 前桥右排气阀 */
    VALVE_LFO    = 6u,   /* 前桥左排气阀 */
    VALVE_LFI    = 7u,   /* 前桥左进气阀 */
    /* 芯片 3 (U12): 21口 + TR_ASR */
    VALVE_21IN   = 8u,   /* 21口 进气阀 (原 TRFIN) */
    VALVE_21OUT  = 9u,   /* 21口 排气阀 (原 TRFOUT) */
    VALVE_TR_ASR = 10u,  /* TR_ASR 阀 (U12 IN3/IN4 并接 P18.6, OUT3/OUT4 并联; 独占 ST34) */
    /* 芯片 4 (U13): 22口 */
    VALVE_22IN   = 11u,  /* 22口 进气阀 (原 FA_ASR) */
    VALVE_22OUT  = 12u,  /* 22口 排气阀 (原 DA_ASR) */
    /* 芯片 5 (U19): 辅助桥阀 */
    VALVE_RXI    = 13u,  /* 辅助桥右进气阀 */
    VALVE_RXO    = 14u,  /* 辅助桥右排气阀 */
    VALVE_LXO    = 15u,  /* 辅助桥左排气阀 */
    VALVE_LXI    = 16u,  /* 辅助桥左进气阀 */
} valve_id_t;

/* ========================================================================== */
/*  故障状态结构体 (开路已实现, 短路由外部电路填充)                             */
/* ========================================================================== */

typedef enum
{
    VALVE_FAULT_NONE  = 0u,   /* 无故障 */
    VALVE_FAULT_OPEN  = 1u,   /* 开路 (线圈断线) */
    VALVE_FAULT_SHORT = 2u,   /* 短路 (由外部电路填充) */
} bts724g_fault_type_t;

typedef struct
{
    bool bts724g_fault_open[VALVE_NUM_TOTAL];    /* 开路故障 (本模块实现) */
    bool bts724g_fault_short[VALVE_NUM_TOTAL];   /* 短路故障 (由 FAR_CK 电路填充) */
    bts724g_fault_type_t bts724g_fault_type[VALVE_NUM_TOTAL];  /* 综合故障类型 */
    bts724g_fault_type_t bts724g_chip_fault_type[BTS724G_NUM_CHIPS]; /* 芯片级故障: 全阀同类型才报 */
    uint8_t              bts724g_chip_any_fault;                     /* 任一芯片有故障标志 (psw_data 读) */
} bts724g_fault_status_t;

/* 全局故障状态 (外部模块读取, ValveDiag_Process() 写入) */
extern volatile bts724g_fault_status_t g_bts724g_fault_status;

/* SysTick 1ms 节拍计数器 (main_cm4.c Timer10ms_Handler 维护, 每 10ms 加 10) */
extern volatile uint32_t g_systick_ms;

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

/**
 * @brief   初始化所有 BTS724G GPIO 引脚。
 *          INx → 推挽输出 (初始 Low, 阀关闭)
 *          STx → 数字输入 + 外部 5.1k 上拉到 VCC (MCU 设为 HIGHZ, 不使用内部 PULLUP)
 */
void Bts724g_Init(void);

/**
 * @brief   设置阀的开关状态 (正常运行中调用)。
 * @param   valve_id  阀索引 (valve_id_t)
 * @param   on         true = 打开, false = 关闭
 */
void Bts724g_SetValve(valve_id_t valve_id, bool on);

/**
 * @brief   查询阀的开路故障状态。
 * @param   valve_id  阀索引 (valve_id_t)
 * @return  true = 开路故障, false = 正常
 */
bool Bts724g_IsOpenFault(valve_id_t valve_id);

/**
 * @brief   查询阀的短路故障状态 (由外部电路填充)。
 * @param   valve_id  阀索引 (valve_id_t)
 * @return  true = 短路故障, false = 正常
 */
bool Bts724g_IsShortFault(valve_id_t valve_id);

/**
 * @brief   外部短路信号入口 (由外部短路检测电路调用)。
 *
 * @details 当外部短路检测电路判定某阀短路时，调用此函数填充故障状态。
 *
 * @param[in] valve_id  阀索引
 * @param[in] is_short  true = 检测到短路, false = 短路已清除
 */
void Bts724g_ReportExternalShort(valve_id_t valve_id, bool is_short);

/**
 * @brief   读取 BTS724G ST 引脚电平 (诊断用, 阀必须处于 ON 状态)。
 *
 * @details BTS724G ST 为开漏输出 (线与), 两个通道共享一个 ST 组。
 *          本函数自动查表 g_bts724g_valve_map 找到对应芯片和 ST 组。
 *          ST=High → 正常, ST=Low → 开路故障 (或对地短路)。
 *
 * @param   valve_id  阀索引
 * @return  true = ST High (正常), false = ST Low (故障)
 */
bool Bts724g_ReadST(valve_id_t valve_id);

/**
 * @brief   通知本模块阀的"使用中"状态 (由应用层调用)。
 *
 * @details ValveDiag_Process() 选择诊断阀时会跳过所有 active=true 的阀。
 *          应用层在调用 Bts724g_SetValve(vid, on) 的同时调用本函数即可，
 *          无需考虑时序，仅检查最新通知值。
 *
 * @param   valve_id  阀索引
 * @param   active    true = 阀工作中 (跳过诊断), false = 空闲 (可诊断)
 */
void Bts724g_NotifyValveActive(valve_id_t valve_id, bool active);

/**
 * @brief   查询本模块记录的"阀使用中"标志。
 * @param   valve_id  阀索引
 * @return  true = 应用层已通知该阀正在工作
 */
bool Bts724g_IsValveActive(valve_id_t valve_id);

/* ========================================================================== */
/*  运行时诊断 (无状态, 主循环每 10ms 调用一次)                                 */
/* ========================================================================== */

/**
 * @brief   阀诊断主函数 (非阻塞, 主循环每 10ms 调用一次)。
 *
 * @details 每 40ms 执行 1 步 (4 步轮转: 阀开路 → 阀短路 → 芯片开路 → 芯片短路), 每步测 1 个阀/芯片。
 *          每步驱动后延时稳定 (VALVE_ON_SETTLE_US) 再读一次 ST/FAR, 消抖靠跨步上下计数。
 *          17 阀 × 2 步 + 5 芯片 × 2 步 = 44 步 ≈ 1.76s 全覆盖。
 *
 * @note    让路条件 (直接 return, 不驱动阀与低边):
 *            1) RTEfValWssTestForbit != 0 (ASW 阀/WSS 测试模式, 低边归 ASW 控制)
 *            2) PSWvIgn 不在 180~320 (继电器刚闭合/掉电, 测量无意义)
 *          另外单阀/单芯片级还会检查 Bts724g_NotifyValveActive() 与故障守卫。
 */
void ValveDiag_Process(void);

/* ========================================================================== */
/*  调试快照开关 (用于 Live Watch)                                             */
/*                                                                            */
/*  1 = 启用 ValveDiag_DebugDump() 和 g_valve_diag_debug                       */
/*  0 = 发布版本编译时移除                                                      */
/*                                                                            */
/*  切换方法:                                                                  */
/*    IAR: Project → Options → C/C++ Compiler → Preprocessor                   */
/*          在 Defined symbols 中添加 VALVE_DIAG_DEBUG=1                       */
/*    或: 直接修改下面的 #ifndef 默认值                                        */
/* ========================================================================== */
#ifndef VALVE_DIAG_DEBUG
#define VALVE_DIAG_DEBUG   0u   /* 默认 1: 开启调试; 发布时改为 0 */
#endif

#if (VALVE_DIAG_DEBUG != 0u)
typedef struct {
    uint32_t systick_ms;                    /* g_systick_ms 快照              */
    uint8_t  reserved;                      /* 保留 (原 step, 现恒为 0)       */
    uint8_t  vid;                           /* 当前被测阀 (0~16)              */
    uint8_t  st_on_raw[3];                  /* ON 态 3 次 ST 读取 (短检用)    */
    uint8_t  st_off_raw[3];                 /* OFF 态 3 次 ST 读取 (开检用)   */
    uint8_t  far_high;                      /* 最近 FAR 判定 (1=High/短路)    */
    uint8_t  far_hi_max;                    /* 保留 (原最大连续 High 次数)     */
    uint8_t  far_scan_count;                /* 最近扫描总读数次数              */
    uint8_t  far_raw_a;                     /* FAR_CK2 P11.1 即时电平 (0/1)  */
    uint8_t  far_raw_b;                     /* FAR_CK  P17.2 即时电平 (0/1)  */
    uint8_t  active[VALVE_NUM_TOTAL];       /* s_valve_active 镜像            */
    uint32_t call_count;                    /* 累计调用次数 (心跳)            */
} valve_diag_debug_t;

extern volatile valve_diag_debug_t g_valve_diag_debug;
void ValveDiag_DebugDump(void);
#endif /* VALVE_DIAG_DEBUG */

/* ========================================================================== */
/*  短路检测 (基于 FAR_CK / FAR_CK2 汇聚电路)                                  */
/*                                                                            */
/*  电路原理 (参见原理图):                                                     */
/*    A 组: U6(4阀)+U9(4阀)=8阀 → FAR_CK2 (P11.1)                            */
/*    B 组: U19(4阀)+U12(3阀)+U13(2阀)=9阀 → FAR_CK (P17.2)                  */
/*                                                                            */
/*  单阀 ON (其余 OFF) 时:                                                     */
/*    - 正常 ON: 三极管饱和导通 → FAR_CK = Low (~0.27V)                         */
/*    - 对地短路: 三极管截止 → FAR_CK = High (~3V)                             */
/*                                                                            */
/*  检测: GPIO 数字输入 (TTL 阈值: VIH=2.0V, VIL=0.8V)                         */
/*    同组全关 → 仅开目标阀 → 320μs 稳定 → 读一次 FAR → High 判短路             */
/*    阀级 3 次上下计数消抖后写入 g_bts724g_fault_status.bts724g_fault_short[]  */
/* ========================================================================== */

typedef enum
{
    SHORT_GROUP_A = 0u,   /* A 组: P11.1 (FAR_CK2), U6+U9 共 8 阀 */
    SHORT_GROUP_B = 1u,   /* B 组: P17.2 (FAR_CK),  U19+U12+U13 共 9 阀 */
    SHORT_GROUP_NUM
} short_group_id_t;

/* GPIO 引脚: 9013 集电极 → MCU 数字输入 (TTL 阈值) */
#define FAR_GPIO_PORT_A         GPIO_PRT11      /* P11.1 = FAR_CK2, A 组 */
#define FAR_GPIO_PIN_A          1u
#define FAR_GPIO_PORT_B         GPIO_PRT17      /* P17.2 = FAR_CK,  B 组 */
#define FAR_GPIO_PIN_B          2u

/**
 * @brief   初始化 P11.1 (FAR_CK2) / P17.2 (FAR_CK) 为 GPIO 数字输入 (TTL 阈值, HIGHZ)。
 */
void ValveShort_Init(void);

/**
 * @brief   GPIO 读取短路组的 FAR_CK 电平。
 * @return  true = High (疑似短路), false = Low (正常)
 */
bool ValveShort_ReadGroup(short_group_id_t group);

#endif /* BTS724G_H */
