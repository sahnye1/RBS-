/**
 * @file    PSWfactory.c
 * @brief   工厂模式框架 — 模式判定 + 命令处理 (不含阀动作)
 *
 * @details 执行流程 (PSWfactory 每 10ms 调用, 位置 PSW_10ms_deal ⑧.7):
 *          ① 读模式: RTEfTestMode == PSWFACTORY_MODE(1) 才继续
 *          ② 模式退出: 测试进行中则同步恢复数据, 否则复位为空闲
 *          ③ 测试推进: 测试进行中每 10ms 处理一批 (64 字节)
 *          ④ 命令处理: 读 RTEfFactoryCmd → 启动测试 → 消费命令
 *
 * @note    · EEPROM 全片测试: 512 字节分 8 批 (每批 64), 流程为
 *            写 0x5A(8批) → 写 0xA5(8批) → 校验(8批) → 恢复原值(8批)
 *            0x5A=01011010 与 0xA5=10100101 互补, 覆盖每一位的写 1 / 写 0 能力。
 *          · 测试前备份 512 字节, 测试后恢复, 不破坏 ERR_CODE/ASW/PSW/COM/BOOT 数据。
 *          · 状态经 RTEfFactoryResult 上报: 0=空闲 1=测试中 2=成功 3=失败。
 *          · 本模块不含任何阀动作 (工厂模式不打阀), 不 include bts724g.h。
 *          · 命令/结果变量当前临时定义在本文件, 待 RTE.h 正式加入后迁移。
 */

#include "cmn.h"
#include "RTE.h"
#include "eeprom.h"
#include "PSWoem.h"
#include "PSWfactory.h"
#include <string.h>

/* ========================================================================== */
/*  0. RTE 命令/结果变量 (临时定义 — 待 RTE.h 正式加入同名 extern 后移至 RTE.c)  */
/* ========================================================================== */
uint8_t RTEfFactoryCmd    = PSWFACTORY_CMD_NONE;    /* 工厂命令字     */
uint8_t RTEfFactoryResult = PSWFACTORY_RESULT_IDLE; /* 执行状态/结果  */

/* ========================================================================== */
/*  1. 常量与运行时状态                                                          */
/* ========================================================================== */

#define EE_TEST_TOTAL      512u    /* EEPROM 总容量 (字节)                          */
#define EE_TEST_BATCH      64u     /* 每批写入/校验字节数                           */
#define EE_TEST_BATCHES    (EE_TEST_TOTAL / EE_TEST_BATCH)   /* 8 批                */

/* 测试阶段 */
#define EE_PHASE_IDLE      0u      /* 空闲                                          */
#define EE_PHASE_WRITE_5A  1u      /* 全片写 0x5A                                   */
#define EE_PHASE_WRITE_A5  2u      /* 全片写 0xA5                                   */
#define EE_PHASE_VERIFY    3u      /* 全片校验 == 0xA5                              */
#define EE_PHASE_RESTORE   4u      /* 全片恢复备份                                  */

static uint8_t s_factory_last_mode = PSWFACTORY_MODE_NORMAL;  /* 上一周期模式      */
static uint8_t s_ee_phase = EE_PHASE_IDLE;   /* 当前测试阶段                        */
static uint8_t s_ee_batch = 0u;              /* 当前批号 0~7                        */
static uint8_t s_ee_backup[EE_TEST_TOTAL];   /* 原数据备份 (512 字节)               */

/* ========================================================================== */
/*  2. 命令处理函数                                                             */
/* ========================================================================== */

/** @brief 当前批的起始地址 */
static uint16_t ee_batch_addr(void)
{
    return (uint16_t)((uint16_t)s_ee_batch * EE_TEST_BATCH);
}

/** @brief 当前批写满指定字节
 *  @return true = 写入并校验成功
 */
static bool ee_write_batch(uint8_t fill)
{
    uint8_t buf[EE_TEST_BATCH];

    memset(buf, fill, EE_TEST_BATCH);
    return (Eeprom_WriteBlock(ee_batch_addr(), buf, EE_TEST_BATCH) == EE_TEST_BATCH);
}

/** @brief 当前批读回校验 (全部等于 fill 才通过) */
static bool ee_verify_batch(uint8_t fill)
{
    uint8_t buf[EE_TEST_BATCH];
    uint8_t i;

    Eeprom_ReadBlock(ee_batch_addr(), buf, EE_TEST_BATCH);
    for (i = 0u; i < EE_TEST_BATCH; i++)
    {
        if (buf[i] != fill) { return false; }
    }
    return true;
}

/** @brief 当前批恢复备份数据 */
static bool ee_restore_batch(void)
{
    const uint16_t addr = ee_batch_addr();
    return (Eeprom_WriteBlock(addr, &s_ee_backup[addr], EE_TEST_BATCH) == EE_TEST_BATCH);
}

/** @brief 启动全片测试: 备份原数据 → 进入写 0x5A 阶段 */
static void factory_ee_start(void)
{
    Eeprom_ReadBlock(0u, s_ee_backup, EE_TEST_TOTAL);   /* 备份 512 字节 */

    s_ee_phase = EE_PHASE_WRITE_5A;
    s_ee_batch = 0u;
    RTEfFactoryResult = PSWFACTORY_RESULT_EE_TESTING;
}

/** @brief 测试推进一批 (每 10ms 一次)
 *  @note  阶段流转: 写0x5A(8批) → 写0xA5(8批) → 校验(8批) → 恢复(8批);
 *         任一步失败 → 置失败并转恢复, 恢复完仍保持失败状态。
 */
static void factory_ee_tick(void)
{
    bool ok = true;

    switch (s_ee_phase)
    {
    case EE_PHASE_WRITE_5A: ok = ee_write_batch(0x5Au);    break;
    case EE_PHASE_WRITE_A5: ok = ee_write_batch(0xA5u);    break;
    case EE_PHASE_VERIFY:   ok = ee_verify_batch(0xA5u);   break;
    case EE_PHASE_RESTORE:  ok = ee_restore_batch();       break;
    default:
        s_ee_phase = EE_PHASE_IDLE;
        return;
    }

    if (!ok)
    {
        if (s_ee_phase == EE_PHASE_RESTORE)
        {
            /* 恢复也失败: 直接结束 */
            s_ee_phase = EE_PHASE_IDLE;
            RTEfFactoryResult = PSWFACTORY_RESULT_EE_FAIL;
        }
        else
        {
            /* 其他阶段失败: 置失败, 转恢复 (尽量还原数据) */
            RTEfFactoryResult = PSWFACTORY_RESULT_EE_FAIL;
            s_ee_phase = EE_PHASE_RESTORE;
            s_ee_batch = 0u;
        }
        return;
    }

    /* 本批完成, 推进到下一批 */
    if (++s_ee_batch >= (uint8_t)EE_TEST_BATCHES)
    {
        s_ee_batch = 0u;
        s_ee_phase++;
        if (s_ee_phase > EE_PHASE_RESTORE)
        {
            /* 恢复完成: 结束; 全程无失败则置成功 */
            s_ee_phase = EE_PHASE_IDLE;
            if (RTEfFactoryResult == PSWFACTORY_RESULT_EE_TESTING)
            {
                RTEfFactoryResult = PSWFACTORY_RESULT_EE_OK;
            }
        }
    }
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

    /* ① 模式退出: 测试进行中则同步恢复全部数据, 否则复位为空闲 */
    if ((s_factory_last_mode == PSWFACTORY_MODE_FACTORY) && (mode != PSWFACTORY_MODE_FACTORY))
    {
        if (s_ee_phase != EE_PHASE_IDLE)
        {
            /* 中途退出: 同步恢复 512 字节 (Eeprom_WriteBlock 内部喂狗, 不会触发看门狗) */
            for (s_ee_batch = 0u; s_ee_batch < (uint8_t)EE_TEST_BATCHES; s_ee_batch++)
            {
                (void)ee_restore_batch();
            }
        }
        s_ee_phase = EE_PHASE_IDLE;
        s_ee_batch = 0u;
        RTEfFactoryResult = PSWFACTORY_RESULT_IDLE;
    }
    s_factory_last_mode = mode;

    /* ② 非工厂模式不接管 */
    if (mode != PSWFACTORY_MODE_FACTORY)
    {
        return;
    }

    /* ③ 测试进行中: 每 10ms 推进一批 (64 字节) */
    if (s_ee_phase != EE_PHASE_IDLE)
    {
        factory_ee_tick();
        return;
    }

    /* ④ 命令处理: 收到命令启动测试 */
    cmd = RTEfFactoryCmd;
    if (cmd == PSWFACTORY_CMD_NONE)
    {
        return;
    }
    RTEfFactoryCmd = PSWFACTORY_CMD_NONE;    /* 消费, 防止重复启动 */

    switch (cmd)
    {
    case PSWFACTORY_CMD_EEPROM_5A_A5:
        factory_ee_start();
        break;

    default:
        break;
    }
}
