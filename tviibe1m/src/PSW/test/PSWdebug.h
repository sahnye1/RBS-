/**
 * @file    PSWdebug.h
 * @brief   测试用例框架入口 — 声明 (实现见 PSWdebug.c)
 *
 * @note    每 10ms 调用一次: 读取 RTEfDbgMsgSW3 (CAN 下发的测试编号) → 匹配用例注册表
 *          → 注册并周期执行; 调用点在 test_deal.c 的 TestDeal_Late(), 受 TEST_DEAL_ENABLE 控制。
 */
#ifndef PSWDEBUG_H
#define PSWDEBUG_H

/** @brief 测试用例框架入口 (每 10ms 调用一次) */
void PSWdebug(void);

#endif /* PSWDEBUG_H */
