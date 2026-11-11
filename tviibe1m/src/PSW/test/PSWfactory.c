/**
 * @file    PSWfactory.c
 * @brief   工厂模式框架 — 模式判定 + 命令处理 (不含阀动作)
 *
 * @details 执行流程 (PSWfactory 每 10ms 调用, 位置 PSW_10ms_deal ⑧.7):
 *          ① 读模式: RTEfTestMode == PSWFACTORY_MODE(1) 才继续
 *          ② 模式退出: 上一周期是工厂模式、本周期不是 → 复位内部状态
 *          ③ 命令处理: 读 RTEfFactoryCmd → 执行 → 结果写 RTEfFactoryResult → 消费命令
 *             - PSWFACTORY_CMD_EEPROM_5A_A5: EEPROM 写 0x5A/0xA5, 检验写1/写0 能力
 *
 * @note    · 本模块不含任何阀动作 (工厂模式不打阀), 不 include bts724g.h。
 *          · EEPROM 测试用未用区 0x160 (0x160~0x1FF 空闲), 不影响各分区数据;
 *            测试后恢复原值。
 *          · 命令/结果变量当前临时定义在本文件, 待 RTE.h 正式加入后迁移。
 *          · 工厂模式属产品功能 (发布版保留), 非工厂模式下立即返回。
 */

#include "cmn.h"
#include "RTE.h"
#include "eeprom.h"
#include "PSWoem.h"
#include "PSWfactory.h"

/* ========================================================================== */
/*  0. RTE 命令/结果变量 (临时定义 — 待 RTE.h 正式加入同名 extern 后移至 RTE.c)  */
/* ========================================================================== */
uint8_t RTEfFactoryCmd    = PSWFACTORY_CMD_NONE;   /* 工厂命令字   */
uint8_t RTEfFactoryResult = PSWFACTORY_RESULT_OK;  /* 命令执行结果 */

/* ========================================================================== */
/*  1. 常量与运行时状态                                                          */
/* ========================================================================== */

/* EEPROM 测试地址: 0x160~0x1FF 为未用区 (在 ERR_CODE/ASW/PSW/COM/BOOT 之后) */
#define FACTORY_EE_TEST_ADDR     0x0160u

static uint8_t s_factory_last_mode = PSWFACTORY_MODE_NORMAL;  /* 上一周期模式 (判进入/退出) */

/* ========================================================================== */
/*  2. 命令处理函数                                                             */
/* ========================================================================== */

/** @brief EEPROM 写 0x5A / 0xA5 检验写 1 / 写 0 能力 (收到命令直写一遍)
 *  @return PSWFACTORY_RESULT_*
 *  @note   0x5A=01011010 与 0xA5=10100101 互补, 两次写入覆盖每一位的 1/0;
 *          写完恢复原值, 不破坏 EEPROM 数据。
 */
static uint8_t factory_eeprom_5a_a5_test(void)
{
    const uint8_t orig = Eeprom_ReadByte(FACTORY_EE_TEST_ADDR);
    uint8_t result = PSWFACTORY_RESULT_OK;

    /* 写 0x5A: 检验 bit1/3/4/6 写 1、bit0/2/5/7 写 0 */
    if ((!Eeprom_WriteByte(FACTORY_EE_TEST_ADDR, 0x5Au)) ||
        (Eeprom_ReadByte(FACTORY_EE_TEST_ADDR) != 0x5Au))
    {
        result = PSWFACTORY_RESULT_EE_5A_FAIL;
    }
    /* 写 0xA5: 互补模式, 检验反向位 */
    else if ((!Eeprom_WriteByte(FACTORY_EE_TEST_ADDR, 0xA5u)) ||
             (Eeprom_ReadByte(FACTORY_EE_TEST_ADDR) != 0xA5u))
    {
        result = PSWFACTORY_RESULT_EE_A5_FAIL;
    }

    /* 恢复原值 */
    (void)Eeprom_WriteByte(FACTORY_EE_TEST_ADDR, orig);

    return result;
}

/* ========================================================================== */
/*  3. 10ms 入口                                                                */
/* ========================================================================== */

/** @brief 工厂模式入口 (每 10ms 调用一次)
 *  @note  非工厂模式时立即返回。
 */
void PSWfactory(void)
{
    uint8_t mode = PSWmode_GetMode();
    uint8_t cmd;

    /* ① 模式退出: 复位内部状态 */
    if ((s_factory_last_mode == PSWFACTORY_MODE_FACTORY) && (mode != PSWFACTORY_MODE_FACTORY))
    {
        RTEfFactoryResult = PSWFACTORY_RESULT_OK;
    }
    s_factory_last_mode = mode;

    /* ② 非工厂模式不接管 */
    if (mode != PSWFACTORY_MODE_FACTORY)
    {
        return;
    }

    /* ③ 命令处理: 收到命令直写一遍 */
    cmd = RTEfFactoryCmd;
    if (cmd == PSWFACTORY_CMD_NONE)
    {
        return;
    }
    RTEfFactoryCmd = PSWFACTORY_CMD_NONE;    /* 消费, 防止重复执行 */

    switch (cmd)
    {
    case PSWFACTORY_CMD_EEPROM_5A_A5:
        RTEfFactoryResult = factory_eeprom_5a_a5_test();
        break;

    default:
        RTEfFactoryResult = PSWFACTORY_RESULT_OK;
        break;
    }
}
