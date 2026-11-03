/**
 * @file    PSWdebug.h
 * @brief   测试用例框架入口 — 声明 (实现见 PSWdebug.c)
 *
 * @note    每 10ms 调用一次 (cmn.c 的 PSW_10ms_deal ⑧.5 步):
 *          读取 RTEfDbgMsgSW3 (UDS 2E 99 99 XX 下发的测试编号) → 匹配用例注册表 → 注册并周期执行。
 *          编号 0x00~0xA1 = 功能测试; 0xB0~0xBB = 底层调试打印 (PSW/RTE 单点 / 阀波形 / 阀诊断明细)。
 */
#ifndef PSWDEBUG_H
#define PSWDEBUG_H

/** @brief 测试用例框架入口 (每 10ms 调用一次) */
void PSWdebug(void);

#endif /* PSWDEBUG_H */
