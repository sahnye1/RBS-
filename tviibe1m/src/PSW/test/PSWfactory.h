/**
 * @file    PSWfactory.h
 * @brief   工厂模式框架 — 声明 (实现见 PSWfactory.c)
 *
 * @note    每 10ms 调用一次 (cmn.c 的 PSW_10ms_deal ⑧.7 步, 紧跟 PSWoem 之后):
 *          模式判定: RTEfTestMode == PSWFACTORY_MODE(1) 时生效。
 *          本模块不含任何电磁阀动作 (工厂模式不打阀)。
 *
 * @note    命令通道: RTEfDbgMsgSW/SW1/SW2 属 COM/ASW 层, PSW 不占用;
 *          工厂模式专用变量待 RTE 增加后再接入 (当前只有模式判定)。
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
/*  对外接口                                                                    */
/* ========================================================================== */

/** @brief 工厂模式入口 (每 10ms 调用一次, 不含任何阀动作) */
void PSWfactory(void);

#endif /* PSWFACTORY_H */
