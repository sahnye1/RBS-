/**
 * @file    PSWoem.c
 * @brief   OEM 模式动作框架 — 模式判定 + 命令分派 + 阀动作
 *
 * @details 执行流程 (PSWoem 每 10ms 调用, 位置 PSW_10ms_deal ⑧.6):
 *          ① 读模式: RTEfTestMode == PSWOEM_MODE_OEM(2) 才继续
 *          ② 模式退出: 上一周期是 OEM、本周期不是 → 关闭本模块打开的所有阀
 *          ③ 命令分派: 读命令字 → 查 s_oem_cmd_list[] → 执行处理函数 → 消费命令
 *
 * @note    · 阀动作统一走 Bts724g_SetValve + Bts724g_NotifyValveActive(active=1),
 *            通知后阀诊断会跳过该阀, 避免 OEM 打阀与阀诊断互相干扰。
 *          · 本模块打开过的阀记在 s_oem_open_mask, 退出 OEM 模式时统一关闭恢复默认。
 *          · 命令是一次性消费 (执行后置 0xFF); 需要跨周期动作 (建压/保压等) 的处理函数
 *            可自带静态状态, 由每 10ms 的调用推进。
 *          · 命令通道变量暂用 RTEfDbgMsgSW / SW1 / SW2, 只经 PSWmode_GetCmd() /
 *            PSWmode_GetArg1() / PSWmode_GetArg2() / PSWmode_AckCmd() 访问;
 *            通讯层确定正式变量后改这 4 个函数即可, 命令表与处理函数不用动。
 *          · 模式变量暂按用户要求用 RTEfTestMode, 同理集中在 PSWmode_GetMode() 一处;
 *            该通道由工厂模式 (PSWfactory.c) 共用。
 */

#include "cmn.h"
#include "bts724g.h"
#include "RTE.h"
#include "PSWoem.h"

/* ========================================================================== */
/*  1. 框架常量与类型                                                           */
/* ========================================================================== */

/** @brief 命令处理函数原型 (p1/p2 = 通讯层下发的两个参数) */
typedef void (*OemCmdFunc)(uint8_t p1, uint8_t p2);

/** @brief 命令表表项 */
typedef struct
{
    uint8_t    cmd;      /* 命令字 (RTEfDbgMsgSW)  */
    OemCmdFunc func;     /* 对应处理函数           */
} oem_cmd_item_t;

/* ========================================================================== */
/*  2. 运行时状态                                                               */
/* ========================================================================== */

static uint8_t  s_oem_last_mode = PSWOEM_MODE_NORMAL;  /* 上一周期模式 (判进入/退出)      */
static uint32_t s_oem_open_mask = 0u;                  /* 本模块打开的阀位图 (bit=valve_id);
                                                        * 按 32 位取: 阀数上限 17 (ABSESC) */

/* ========================================================================== */
/*  3. 模式/命令通道读写接口 (通讯层变量一旦确定, 只改本节)                     */
/*                                                                            */
/*  由 OEM 模式 (本文件) 与工厂模式 (PSWfactory.c) 共用 — 两模式互斥,           */
/*  共用同一组命令变量不会互相抢命令; 声明见 PSWoem.h。                          */
/* ========================================================================== */

/** @brief 读当前模式 (RTEfTestMode: 0 正常 / 1 工厂 / 2 OEM) */
uint8_t PSWmode_GetMode(void)
{
    return RTEfTestMode;
}

/** @brief 读命令字 */
uint8_t PSWmode_GetCmd(void)
{
    return RTEfDbgMsgSW;
}

/** @brief 读参数1 */
uint8_t PSWmode_GetArg1(void)
{
    return RTEfDbgMsgSW1;
}

/** @brief 读参数2 */
uint8_t PSWmode_GetArg2(void)
{
    return RTEfDbgMsgSW2;
}

/** @brief 消费命令 (置空闲, 防止同一命令被反复执行) */
void PSWmode_AckCmd(void)
{
    RTEfDbgMsgSW = PSWOEM_CMD_NONE;
}

/* ========================================================================== */
/*  4. 阀动作                                                                   */
/* ========================================================================== */

/** @brief 关闭全部阀并清空标记 (退出 OEM 模式时调用)
 *  @note  对所有阀调一次 SetValve(false), 已关闭的阀重复关闭无副作用。
 */
static void oem_all_off(void)
{
    uint8_t vid;

    for (vid = 0u; vid < VALVE_NUM_TOTAL; vid++)
    {
        Bts724g_SetValve((valve_id_t)vid, false);
        Bts724g_NotifyValveActive((valve_id_t)vid, false);
    }
    s_oem_open_mask = 0u;
}

/** @brief 打开指定阀 (并通知阀诊断跳过该阀)
 *  @param vid 阀索引 (valve_id_t, 0~VALVE_NUM_TOTAL-1), 越界忽略
 */
static void oem_valve_on(uint8_t vid)
{
    if (vid >= VALVE_NUM_TOTAL)
    {
        return;
    }
    Bts724g_NotifyValveActive((valve_id_t)vid, true);
    Bts724g_SetValve((valve_id_t)vid, true);
    s_oem_open_mask |= (uint32_t)(1ul << vid);
}

/** @brief 关闭指定阀 (并解除阀诊断跳过)
 *  @param vid 阀索引 (valve_id_t, 0~VALVE_NUM_TOTAL-1), 越界忽略
 */
static void oem_valve_off(uint8_t vid)
{
    if (vid >= VALVE_NUM_TOTAL)
    {
        return;
    }
    Bts724g_SetValve((valve_id_t)vid, false);
    Bts724g_NotifyValveActive((valve_id_t)vid, false);
    s_oem_open_mask &= (uint32_t)~(uint32_t)(1ul << vid);
}

/* ========================================================================== */
/*  5. 命令处理函数                                                             */
/*                                                                            */
/*  新增命令: 在此加一个 static 处理函数, 再到 s_oem_cmd_list[] 加一条表项。     */
/* ========================================================================== */

/** @brief 0x00 关闭全部阀 — 参数未用 */
static void oem_cmd_all_off(uint8_t p1, uint8_t p2)
{
    (void)p1;
    (void)p2;
    oem_all_off();
}

/** @brief 0x01 打开指定阀 — p1 = valve_id */
static void oem_cmd_valve_on(uint8_t p1, uint8_t p2)
{
    (void)p2;
    oem_valve_on(p1);
}

/** @brief 0x02 关闭指定阀 — p1 = valve_id */
static void oem_cmd_valve_off(uint8_t p1, uint8_t p2)
{
    (void)p2;
    oem_valve_off(p1);
}

/* ========================================================================== */
/*  6. 命令表                                                                   */
/* ========================================================================== */

static const oem_cmd_item_t s_oem_cmd_list[] =
{
    { PSWOEM_CMD_ALL_OFF,  oem_cmd_all_off  },
    { PSWOEM_CMD_VALVE_ON, oem_cmd_valve_on },
    { PSWOEM_CMD_VALVE_OFF, oem_cmd_valve_off },
};

#define PSWOEM_CMD_NUM   (sizeof(s_oem_cmd_list) / sizeof(s_oem_cmd_list[0]))

/* ========================================================================== */
/*  7. 10ms 入口                                                                */
/* ========================================================================== */

/** @brief OEM 模式动作框架入口 (每 10ms 调用一次)
 *  @note  非 OEM 模式时只做一次退出恢复, 不读取/不消费命令字,
 *         避免占用通讯层在同组变量上的其它用途。
 */
void PSWoem(void)
{
    uint8_t mode = PSWmode_GetMode();
    uint8_t cmd;
    uint8_t i;

    /* ① 模式退出: 关闭本模块打过的阀, 恢复默认状态 */
    if ((s_oem_last_mode == PSWOEM_MODE_OEM) && (mode != PSWOEM_MODE_OEM))
    {
        oem_all_off();
    }
    s_oem_last_mode = mode;

    /* ② 非 OEM 模式不接管命令 */
    if (mode != PSWOEM_MODE_OEM)
    {
        return;
    }

    /* ③ 命令分派 */
    cmd = PSWmode_GetCmd();
    if (cmd == PSWOEM_CMD_NONE)
    {
        return;
    }

    for (i = 0u; i < (uint8_t)PSWOEM_CMD_NUM; i++)
    {
        if (s_oem_cmd_list[i].cmd == cmd)
        {
            s_oem_cmd_list[i].func(PSWmode_GetArg1(), PSWmode_GetArg2());
            break;
        }
    }

    /* ④ 消费命令 (未匹配也清, 防止同一未知命令每周期重入) */
    PSWmode_AckCmd();
}
