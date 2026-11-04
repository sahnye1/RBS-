/**
 * @file    PSWfactory.c
 * @brief   工厂模式动作框架 — 模式判定 + 命令分派 (不含任何阀动作)
 *
 * @details 执行流程 (PSWfactory 每 10ms 调用, 位置 PSW_10ms_deal ⑧.7):
 *          ① 读模式: RTEfTestMode == PSWFACTORY_MODE(1) 才继续
 *          ② 模式退出: 上一周期是工厂模式、本周期不是 → 复位本模块内部状态
 *          ③ 命令分派: 读命令字 → 查 s_factory_cmd_list[] → 执行处理函数 → 消费命令
 *
 * @note    · 本模块不含任何阀动作: 不 include bts724g.h, 命令表里也不放阀处理函数。
 *            工厂模式若确需驱动输出, 须先与用户确认, 且与 OEM 模式的阀动作区分开。
 *          · 命令通道与 OEM 模式共用, 统一走 PSWmode_* 接口 (实现见 PSWoem.c 第 3 节);
 *            通讯层新增正式变量后只改那一处, 本模块不用动。
 *          · 命令是一次性消费 (执行后置 0xFF); 需要跨周期动作的处理函数可自带静态状态,
 *            由每 10ms 的调用推进。
 *          · 结果回读 (命令执行结果上报通讯层) 的通道尚未确定: 暂把最后一条命令的参数
 *            存在本模块静态变量里, 便于 Live Watch 观察; 待通讯层给出变量后再加回读函数。
 *          · 工厂模式属产品功能 (发布版保留), 非工厂模式下立即返回。
 */

#include "cmn.h"
#include "RTE.h"
#include "PSWoem.h"
#include "PSWfactory.h"

/* ========================================================================== */
/*  1. 框架常量与类型                                                           */
/* ========================================================================== */

/** @brief 命令处理函数原型 (p1/p2 = 通讯层下发的两个参数) */
typedef void (*FactoryCmdFunc)(uint8_t p1, uint8_t p2);

/** @brief 命令表表项 */
typedef struct
{
    uint8_t        cmd;     /* 命令字 (RTEfDbgMsgSW)  */
    FactoryCmdFunc func;    /* 对应处理函数           */
} factory_cmd_item_t;

/* ========================================================================== */
/*  2. 运行时状态                                                               */
/* ========================================================================== */

static uint8_t  s_factory_last_mode = PSWFACTORY_MODE_NORMAL;  /* 上一周期模式 (判进入/退出) */
static uint16_t s_factory_cmd_cnt   = 0u;                      /* 收到的命令计数 (通道验证)   */
static uint8_t  s_factory_last_arg1 = 0u;                      /* 最后一条命令的参数1         */
static uint8_t  s_factory_last_arg2 = 0u;                      /* 最后一条命令的参数2         */

/* ========================================================================== */
/*  3. 命令处理函数                                                             */
/*                                                                            */
/*  新增命令: 在此加一个 static 处理函数, 再到 s_factory_cmd_list[] 加一条表项,  */
/*            命令字宏加在 PSWfactory.h。                                        */
/*  约束: 处理函数里不得出现电磁阀动作 (Bts724g_SetValve 等)。                    */
/* ========================================================================== */

/** @brief 0x80 空动作 — 只用于确认命令通道与模式判定是否正常, 参数未用 */
static void factory_cmd_nop(uint8_t p1, uint8_t p2)
{
    (void)p1;
    (void)p2;
}

/* ========================================================================== */
/*  4. 命令表                                                                   */
/* ========================================================================== */

static const factory_cmd_item_t s_factory_cmd_list[] =
{
    { PSWFACTORY_CMD_NOP, factory_cmd_nop },
};

#define PSWFACTORY_CMD_NUM   (sizeof(s_factory_cmd_list) / sizeof(s_factory_cmd_list[0]))

/* ========================================================================== */
/*  5. 10ms 入口                                                                */
/* ========================================================================== */

/** @brief 工厂模式动作框架入口 (每 10ms 调用一次)
 *  @note  非工厂模式时只做一次状态复位, 不读取/不消费命令字,
 *         避免与 OEM 模式 (同一组命令变量) 互相干扰。
 */
void PSWfactory(void)
{
    uint8_t mode = PSWmode_GetMode();
    uint8_t cmd;
    uint8_t i;

    /* ① 模式退出: 复位内部状态 (本模块不驱阀, 无需恢复输出) */
    if ((s_factory_last_mode == PSWFACTORY_MODE_FACTORY) && (mode != PSWFACTORY_MODE_FACTORY))
    {
        s_factory_cmd_cnt   = 0u;
        s_factory_last_arg1 = 0u;
        s_factory_last_arg2 = 0u;
    }
    s_factory_last_mode = mode;

    /* ② 非工厂模式不接管命令 */
    if (mode != PSWFACTORY_MODE_FACTORY)
    {
        return;
    }

    /* ③ 命令分派 */
    cmd = PSWmode_GetCmd();
    if (cmd == PSWFACTORY_CMD_NONE)
    {
        return;
    }

    s_factory_last_arg1 = PSWmode_GetArg1();
    s_factory_last_arg2 = PSWmode_GetArg2();
    s_factory_cmd_cnt++;

    for (i = 0u; i < (uint8_t)PSWFACTORY_CMD_NUM; i++)
    {
        if (s_factory_cmd_list[i].cmd == cmd)
        {
            s_factory_cmd_list[i].func(s_factory_last_arg1, s_factory_last_arg2);
            break;
        }
    }

    /* ④ 消费命令 (未匹配也清, 防止同一未知命令每周期重入) */
    PSWmode_AckCmd();
}
