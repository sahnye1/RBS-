/**
 * @file    psw_log.c
 * @brief   底层健康档案 + 错误日志实现
 *
 * @note    写入策略:
 *            - 系统统计: 每次上电合并后写一次 (EEPROM_LOG_BASE, 32 B)
 *            - 错误日志: 某错误"首次出现"时写该条 (8 B), 后续只累加 RAM 计数
 *          → 正常使用下 EEPROM 写入次数极少, 无磨损风险。
 */

#include "cy_project.h"       /* 含 sysreset/cy_sysreset.h (Cy_SysReset_*) */
#include "cy_device_headers.h"
#include <string.h>
#include <stddef.h>
#include "psw_log.h"
#include "psw_data.h"
#include "RTE.h"

/* ========================================================================== */
/*  内部状态                                                                    */
/* ========================================================================== */
static psw_stat_t       s_stat;                    /* 统计 RAM 副本 */
static psw_err_entry_t  s_err[PSW_ERRLOG_MAX];     /* 错误日志 RAM 副本 */

static uint8_t   s_early_reason = PSW_RST_POR;     /* EarlyInit 读到的复位原因 */
static uint32_t  s_uptime_ms    = 0u;              /* 本次上电已运行 (ms) */
static uint16_t  s_tick_10ms    = 0u;              /* 10ms 计数 (凑分钟) */
static uint32_t  s_run_minutes  = 0u;              /* 本次上电已运行 (分钟) */

static uint16_t  s_can_busoff_ram[2];              /* 本次上电 busoff 增量 */
static uint16_t  s_ee_fail_ram   = 0u;             /* 本次上电 EEPROM 写失败增量 */
static uint16_t  s_init_fail_ram = 0u;             /* 本次上电初始化失败位图 */
static uint8_t   s_vmin_ram      = 0u;             /* 本次上电最低电压 */
static uint8_t   s_vmax_ram      = 0u;             /* 本次上电最高电压 */

/* ========================================================================== */
/*  CRC16-CCITT (多项式 0x1021, 初值 0xFFFF)                                     */
/* ========================================================================== */
static uint16_t crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;

    for (uint32_t i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8u);
        for (uint8_t b = 0u; b < 8u; b++)
        {
            crc = ((crc & 0x8000u) != 0u)
                  ? (uint16_t)((uint16_t)(crc << 1u) ^ 0x1021u)
                  : (uint16_t)(crc << 1u);
        }
    }
    return crc;
}

/* ========================================================================== */
/*  1. 上电最早期: 读复位原因 (只存 RAM, 不清除)                                  */
/* ========================================================================== */
void PSWLog_EarlyInit(void)
{
    uint32_t raw = Cy_SysReset_GetResetReason();

    if ((raw & CY_SYSRESET_WDT) != 0u)
    {
        s_early_reason = PSW_RST_WDT;
    }
    else if ((raw & (CY_SYSRESET_ACT_FAULT | CY_SYSRESET_DPSLP_FAULT)) != 0u)
    {
        s_early_reason = PSW_RST_FAULT;
    }
    else if ((raw & CY_SYSRESET_SOFT) != 0u)
    {
        s_early_reason = PSW_RST_SOFT;
    }
    else if ((raw & (CY_SYSRESET_BODVDDD | CY_SYSRESET_BODVDDA | CY_SYSRESET_BODVCCD)) != 0u)
    {
        s_early_reason = PSW_RST_BOD;
    }
    else if (raw == 0u)
    {
        s_early_reason = PSW_RST_POR;
    }
    else
    {
        s_early_reason = PSW_RST_OTHER;
    }
    /* 不清除: 等 PSWLog_Init() 落盘后再清 */
}

/* ========================================================================== */
/*  2. EEPROM 可用后: 读旧统计 → 合并本次增量 → 写回一次                          */
/* ========================================================================== */
void PSWLog_Init(void)
{
    psw_stat_t old;

    /* ---- 2.1 读旧统计 ---- */
    if (Eeprom_ReadBlock(EEPROM_LOG_BASE, (uint8_t *)&old, (uint32_t)sizeof(old))
        != (uint32_t)sizeof(old))
    {
        memset(&old, 0, sizeof(old));
    }
    /* CRC 校验: 失败视为首次上电 (不阻断启动) */
    else if ((old.crc == 0u) ||
             (crc16((uint8_t *)&old, (uint32_t)offsetof(psw_stat_t, crc)) != old.crc))
    {
        memset(&old, 0, sizeof(old));
    }

    /* ---- 2.2 合并本次复位原因 ---- */
    switch (s_early_reason)
    {
        case PSW_RST_POR:   old.por_cnt++;   break;
        case PSW_RST_WDT:   old.wdt_cnt++;   break;
        case PSW_RST_SOFT:  old.soft_cnt++;  break;
        case PSW_RST_FAULT: old.fault_cnt++; break;
        case PSW_RST_BOD:   old.bod_cnt++;   break;
        default:                             break;
    }
    old.last_reason = s_early_reason;

    /* ---- 2.3 合并上次运行期增量 ---- */
    old.run_hours     += (uint32_t)(s_run_minutes / 60u);
    old.can0_busoff   += s_can_busoff_ram[0];
    old.can1_busoff   += s_can_busoff_ram[1];
    old.ee_write_fail += s_ee_fail_ram;
    old.init_fail_mask |= s_init_fail_ram;

    if (s_vmin_ram != 0u)
    {
        if ((old.vmin == 0u) || (s_vmin_ram < old.vmin)) { old.vmin = s_vmin_ram; }
    }
    if (s_vmax_ram > old.vmax) { old.vmax = s_vmax_ram; }

    /* ---- 2.4 软件版本 ---- */
    old.ver_bcd = ((uint32_t)RTEASW_Version[1] << 24)
                | ((uint32_t)RTEASW_Version[2] << 16)
                | ((uint32_t)RTEASW_Version[3] << 8)
                |  (uint32_t)RTEASW_Version[4];

    /* ---- 2.5 CRC + 写回 EEPROM (本次上电唯一一次写) ---- */
    old.crc = crc16((uint8_t *)&old, (uint32_t)offsetof(psw_stat_t, crc));
    (void)Eeprom_WriteBlock(EEPROM_LOG_BASE, (uint8_t *)&old, (uint32_t)sizeof(old));

    /* ---- 2.6 清硬件复位原因 (已落盘) ---- */
    Cy_SysReset_ClearAllResetReasons();

    /* ---- 2.7 载入 RAM 副本 ---- */
    s_stat = old;

    /* ---- 2.8 读历史错误日志到 RAM (作为基线) ---- */
    if (Eeprom_ReadBlock(EEPROM_ERRLOG_BASE, (uint8_t *)s_err, (uint32_t)sizeof(s_err))
        != (uint32_t)sizeof(s_err))
    {
        memset(s_err, 0, sizeof(s_err));
    }
    /* 非法条目 (code 高字节非 0x00~0x0F) 清空 */
    for (uint8_t i = 0u; i < PSW_ERRLOG_MAX; i++)
    {
        if ((s_err[i].code & 0xF000u) != 0u) { memset(&s_err[i], 0, sizeof(s_err[i])); }
    }
}

/* ========================================================================== */
/*  3. 10ms 周期: 运行时间 + 电压跟踪 (只更新 RAM)                                */
/* ========================================================================== */
void PSWLog_Tick(void)
{
    uint16_t v;
    uint8_t  vb;

    s_uptime_ms += 10u;

    if (++s_tick_10ms >= 6000u)       /* 6000 × 10ms = 1 分钟 */
    {
        s_tick_10ms = 0u;
        s_run_minutes++;
    }

    /* 电压 min/max (PSWvIgn: 0.1V, 饱和到 uint8) */
    v  = PSWvIgn;
    vb = (v > 255u) ? 255u : (uint8_t)v;
    if (vb != 0u)
    {
        if ((s_vmin_ram == 0u) || (vb < s_vmin_ram)) { s_vmin_ram = vb; }
        if (vb > s_vmax_ram) { s_vmax_ram = vb; }
    }
}

/* ========================================================================== */
/*  4. 记录错误 (新错误首次出现时写 EEPROM 该条)                                  */
/* ========================================================================== */
void PSWLog_Err(uint16_t code)
{
    uint8_t i;

    if (code == 0u) { return; }

    /* 已有该错误 → RAM 累加, 不写 EEPROM */
    for (i = 0u; i < PSW_ERRLOG_MAX; i++)
    {
        if (s_err[i].code == code)
        {
            if (s_err[i].count < 0xFFFFu) { s_err[i].count++; }
            return;
        }
    }

    /* 新错误 → 找空槽并写 EEPROM */
    for (i = 0u; i < PSW_ERRLOG_MAX; i++)
    {
        if (s_err[i].code == 0u)
        {
            s_err[i].code     = code;
            s_err[i].count    = 1u;
            s_err[i].first_ms = s_uptime_ms;
            (void)Eeprom_WriteBlock((uint16_t)(EEPROM_ERRLOG_BASE + (uint16_t)i * 8u),
                                    (uint8_t *)&s_err[i], 8u);
            return;
        }
    }

    /* 槽位满 → 覆盖第 0 条 (最老) */
    s_err[0].code     = code;
    s_err[0].count    = 1u;
    s_err[0].first_ms = s_uptime_ms;
    (void)Eeprom_WriteBlock(EEPROM_ERRLOG_BASE, (uint8_t *)&s_err[0], 8u);
}

/* ========================================================================== */
/*  5. 计数接口                                                                 */
/* ========================================================================== */
void PSWLog_Busoff(uint8_t ch)
{
    if (ch < 2u)
    {
        if (s_can_busoff_ram[ch] < 0xFFFFu) { s_can_busoff_ram[ch]++; }
    }
}

void PSWLog_InitFail(uint16_t mask)
{
    s_init_fail_ram |= mask;
}

/* ========================================================================== */
/*  6. 查询                                                                     */
/* ========================================================================== */
const psw_stat_t* PSWLog_GetStat(void)
{
    return &s_stat;
}

uint8_t PSWLog_GetErr(psw_err_entry_t *out, uint8_t max)
{
    uint8_t n = 0u;

    if (out == NULL) { return 0u; }

    for (uint8_t i = 0u; (i < PSW_ERRLOG_MAX) && (n < max); i++)
    {
        if (s_err[i].code != 0u) { out[n++] = s_err[i]; }
    }
    return n;
}
