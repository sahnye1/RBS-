/**
 * @file    psw_log.h
 * @brief   底层健康档案 + 错误日志 (复位原因 / 运行时长 / 质量指标 / 返回值异常)
 *
 * @details 目标: 把"掉电/复位后就查不到"的底层信息持久化到 EEPROM, 供台架/售后追溯。
 *
 *          存储布局 (EEPROM 空闲区, 见 eeprom.h):
 *            0x0160  系统统计 psw_stat_t   32 B
 *            0x0180  错误日志 8 条 × 8 B   64 B
 *
 *          写入策略 (关键: 防 EEPROM 磨损):
 *            运行期间  → 只更新 RAM, 不碰 EEPROM
 *            每次上电  → PSWLog_Init() 合并 RAM 增量后写一次 EEPROM
 *          → EEPROM 写入次数 = 上电次数, 正常使用无磨损风险。
 *
 *          读取时序 (关键: 复位原因是"读后即清"的):
 *            board_init 最前 (SoftWD_init 之前) → PSWLog_EarlyInit() 只读原因到 RAM
 *            Eeprom_Init() 之后                → PSWLog_Init() 合并并落盘
 */

#ifndef PSW_LOG_H
#define PSW_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include "eeprom.h"     /* EEPROM_LOG_BASE / EEPROM_ERRLOG_BASE */

/* ========================================================================== */
/*  复位原因                                                                    */
/* ========================================================================== */
typedef enum
{
    PSW_RST_POR   = 0u,   /* 上电 (无特殊原因) */
    PSW_RST_WDT   = 1u,   /* 看门狗超时复位 ★ */
    PSW_RST_SOFT  = 2u,   /* 软件复位 (SYSRESETREQ) */
    PSW_RST_FAULT = 3u,   /* 硬件故障复位 (ACT/DPSLP fault) ★ */
    PSW_RST_BOD   = 4u,   /* 欠压复位 (VDDD/VDDA/VCCD brown-out) ★ */
    PSW_RST_OTHER = 5u,   /* 其他 (XRES / 过压 / 调试器) */
} psw_rst_reason_t;

/* ========================================================================== */
/*  初始化失败位图                                                              */
/* ========================================================================== */
#define PSW_INITFAIL_ADC     (1u << 0)
#define PSW_INITFAIL_SPI     (1u << 1)
#define PSW_INITFAIL_EEPROM  (1u << 2)
#define PSW_INITFAIL_CAN0    (1u << 3)
#define PSW_INITFAIL_CAN1    (1u << 4)
#define PSW_INITFAIL_CLK     (1u << 5)
#define PSW_INITFAIL_GPIO    (1u << 6)

/* ========================================================================== */
/*  错误码 (模块高字节 | 类型低字节)                                            */
/* ========================================================================== */
#define PSW_ERR_EEPROM_WRITE  0x0101u   /* EEPROM 块写失败 */
#define PSW_ERR_EEPROM_READ   0x0102u   /* EEPROM 块读失败 */
#define PSW_ERR_SPI_TIMEOUT   0x0201u   /* SPI 传输超时 */
#define PSW_ERR_ADC_INIT      0x0301u   /* ADC 初始化失败 */
#define PSW_ERR_ADC_TIMEOUT   0x0302u   /* ADC 转换超时 */
#define PSW_ERR_CAN_SEND      0x0401u   /* CAN 发送失败 */
#define PSW_ERR_CAN_BUSOFF    0x0402u   /* CAN busoff */
#define PSW_ERR_VALVE_SET     0x0501u   /* 阀设置失败 */
#define PSW_ERR_CLK_CONFIG    0x0601u   /* 时钟配置失败 */

/* ========================================================================== */
/*  系统统计 (32 B, 存 EEPROM 0x0160)                                           */
/*  ⚠ 字段顺序按 4 字节对齐排, 保证 sizeof == 32 (无 padding)                    */
/* ========================================================================== */
typedef struct
{
    uint32_t run_hours;       /* 累计运行小时 (每次上电累加上次运行时长) */
    uint32_t ver_bcd;         /* 软件版本 BCD (年/月/日/序号) */
    uint16_t por_cnt;         /* 上电复位次数 */
    uint16_t wdt_cnt;         /* ★ 看门狗复位次数 */
    uint16_t soft_cnt;        /* 软件复位次数 */
    uint16_t fault_cnt;       /* ★ 硬件故障复位次数 */
    uint16_t bod_cnt;         /* ★ 欠压复位次数 (电源质量) */
    uint16_t can0_busoff;     /* CAN0 busoff 累计次数 */
    uint16_t can1_busoff;     /* CAN1 busoff 累计次数 */
    uint16_t ee_write_fail;   /* EEPROM 写失败累计次数 */
    uint16_t init_fail_mask;  /* 外设初始化失败位图 (PSW_INITFAIL_*) */
    uint16_t crc;             /* CRC16-CCITT (前 30 字节) */
    uint8_t  vmin;            /* 历史最低电压 (0.1V, 饱和 0~255) */
    uint8_t  vmax;            /* 历史最高电压 (0.1V) */
    uint8_t  last_reason;     /* 最近一次复位原因 (psw_rst_reason_t) */
    uint8_t  rsv;             /* 保留 */
} psw_stat_t;                 /* = 32 B */

/* ========================================================================== */
/*  错误日志条目 (8 B × 8 条 = 64 B, 存 EEPROM 0x0180)                          */
/* ========================================================================== */
typedef struct
{
    uint16_t code;            /* 错误码 (0 = 空槽) */
    uint16_t count;           /* 累计次数 (饱和 0xFFFF) */
    uint32_t first_ms;        /* 首次发生时间 (ms, 相对上电) */
} psw_err_entry_t;            /* = 8 B */

#define PSW_ERRLOG_MAX   8u

/* ========================================================================== */
/*  接口                                                                        */
/* ========================================================================== */

/** @brief 上电最早期调用 (board_init 内, SoftWD_init 之前): 只读复位原因到 RAM
 *  @note  此时 EEPROM/SPI 尚未初始化, 本函数不访问任何外设, 只读 SRSS 寄存器 */
void PSWLog_EarlyInit(void);

/** @brief EEPROM 初始化之后调用: 读旧统计 → 合并本次增量 → 写回 EEPROM 一次
 *  @note  调用后会清除硬件复位原因寄存器 */
void PSWLog_Init(void);

/** @brief 10ms 周期调用: 运行时间累计 + 电压 min/max 跟踪 (只更新 RAM) */
void PSWLog_Tick(void);

/** @brief 记录一次底层错误 (只更新 RAM, 不写 EEPROM) */
void PSWLog_Err(uint16_t code);

/** @brief busoff 计数 (ch: 0=CAN0, 1=CAN1) */
void PSWLog_Busoff(uint8_t ch);

/** @brief 置外设初始化失败位 (PSW_INITFAIL_*) */
void PSWLog_InitFail(uint16_t mask);

/** @brief 查询系统统计 (返回内部 RAM 副本, 只读) */
const psw_stat_t* PSWLog_GetStat(void);

/** @brief 读取错误日志 (跳过空槽)
 *  @param out  输出缓冲 / @param max 最多读取条数
 *  @return 实际条数 */
uint8_t PSWLog_GetErr(psw_err_entry_t *out, uint8_t max);

#endif /* PSW_LOG_H */
