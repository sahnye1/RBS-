/**
 * @file    PSWfactory.h
 * @brief   工厂模式框架 — 声明 (实现见 PSWfactory.c)
 *
 * @note    每 10ms 调用一次 (cmn.c 的 PSW_10ms_deal ⑧.7 步, 紧跟 PSWoem 之后):
 *          模式判定: RTEfTestMode == PSWFACTORY_MODE(1) 时生效。
 *          本模块不含任何电磁阀动作 (工厂模式不打阀)。
 *
 * @note    命令通道 (RTE 变量, 待 RTE.h 正式加入后迁移):
 *            RTEfFactoryCmd    = 命令字 (执行后 PSW 置回 PSWFACTORY_CMD_NONE)
 *            RTEfFactoryResult = 执行结果
 *          工厂模式属产品功能 (发布版保留), 非工厂模式下立即返回。
 */
#ifndef PSWFACTORY_H
#define PSWFACTORY_H

/* ========================================================================== */
/*  模式值 (与 RTEfTestMode 的取值定义一致)                                     */
/* ========================================================================== */
#define PSWFACTORY_MODE_NORMAL      0u    /* 正常模式 — 框架不动作              */
#define PSWFACTORY_MODE_FACTORY     1u    /* 工厂测试模式 — 框架生效            */

/* ========================================================================== */
/*  命令字 (RTEfFactoryCmd 的取值)                                              */
/* ========================================================================== */
#define PSWFACTORY_CMD_NONE         0u    /* 空闲 (执行完由 PSW 置回)           */
#define PSWFACTORY_CMD_EEPROM_5A_A5 1u    /* EEPROM 写 0x5A/0xA5, 检验写1/写0   */

/* ========================================================================== */
/*  执行结果 (RTEfFactoryResult 的取值)                                         */
/* ========================================================================== */
#define PSWFACTORY_RESULT_OK        0u    /* 通过                               */
#define PSWFACTORY_RESULT_EE_5A_FAIL 1u   /* 写 0x5A 失败 / 读回不符            */
#define PSWFACTORY_RESULT_EE_A5_FAIL 2u   /* 写 0xA5 失败 / 读回不符            */

/* ========================================================================== */
/*  RTE 命令/结果变量 (临时定义 — RTE.h 后续会正式加入同名 extern, 届时删除本节)  */
/* ========================================================================== */

/** 工厂命令字: COM/ASW 写入, PSW 执行一次后置回 PSWFACTORY_CMD_NONE */
extern uint8_t RTEfFactoryCmd;

/** 命令执行结果: PSW 写入, COM/ASW 读取 */
extern uint8_t RTEfFactoryResult;

/* ========================================================================== */
/*  对外接口                                                                    */
/* ========================================================================== */

/** @brief 工厂模式入口 (每 10ms 调用一次, 不含任何阀动作) */
void PSWfactory(void);

#endif /* PSWFACTORY_H */
