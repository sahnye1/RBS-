/**
 * @file    PSWfactory.c
 * @brief   工厂模式框架 — 模式判定 (不含阀动作; 命令通道待 RTE 变量确定)
 *
 * @details 执行流程 (PSWfactory 每 10ms 调用, 位置 PSW_10ms_deal ⑧.7):
 *          ① 读模式: RTEfTestMode == PSWFACTORY_MODE(1) 才继续
 *          ② 模式退出: 上一周期是工厂模式、本周期不是 → 复位内部状态
 *          ③ 命令处理: 待 RTE 提供工厂模式专用变量后接入
 *
 * @note    · 本模块不含任何阀动作 (工厂模式不打阀), 不 include bts724g.h。
 *          · 命令通道: RTEfDbgMsgSW/SW1/SW2 属 COM/ASW 层, PSW 不占用;
 *            工厂模式专用变量待 RTE 增加后再接入, 届时只改本文件。
 *          · 工厂模式属产品功能 (发布版保留), 非工厂模式下立即返回。
 */

#include "cmn.h"
#include "RTE.h"
#include "PSWoem.h"
#include "PSWfactory.h"

/* ========================================================================== */
/*  1. 运行时状态                                                               */
/* ========================================================================== */

static uint8_t s_factory_last_mode = PSWFACTORY_MODE_NORMAL;  /* 上一周期模式 (判进入/退出) */

/* ========================================================================== */
/*  2. 10ms 入口                                                                */
/* ========================================================================== */

/** @brief 工厂模式入口 (每 10ms 调用一次)
 *  @note  非工厂模式时立即返回。
 */
void PSWfactory(void)
{
    uint8_t mode = PSWmode_GetMode();

    /* ① 模式退出: 复位内部状态 */
    if ((s_factory_last_mode == PSWFACTORY_MODE_FACTORY) && (mode != PSWFACTORY_MODE_FACTORY))
    {
        /* 当前无内部状态需复位 (命令通道待接入) */
    }
    s_factory_last_mode = mode;

    /* ② 非工厂模式不接管 */
    if (mode != PSWFACTORY_MODE_FACTORY)
    {
        return;
    }

    /* ③ 命令处理: 待 RTE 提供工厂模式专用变量后接入 */
}
