/**
 * @file    test_cfg.h
 * @brief   测试/调试总开关 — 所有测试用例与调试上报的开关集中在本文件
 *
 * @details 用法:
 *          - TEST_DEAL_ENABLE = 0 → 测试层整体不编译 (test_deal.c 给空入口,
 *                                    cmn.c 无需任何改动, 链接照常)
 *          - TEST_DEAL_ENABLE = 1 → 再按下面各分项开关单独开启
 *
 * @note    模块级开关不在此处 (随各自模块走):
 *          - VALVE_DIAG_DEBUG  (bts724g.h)         阀诊断明细输出
 *          - VHT_AUTO_START    (valve_hold_test.h) 阀保压标定是否上电自动跑一次
 */

#ifndef TEST_CFG_H
#define TEST_CFG_H

/* ---- 总开关 ---- */
#define TEST_DEAL_ENABLE        1u   /* 1 = 编译测试层; 0 = 全部测试/调试不编译 */

/* ---- 分项开关 (仅总开关 = 1 时有效) ---- */
#define TEST_VALVE_HOLD         0u   /* 阀保压阶梯标定 (valve_hold_test, 21口)                     */
#define TEST_VALVE_DRV_WAVE     0u   /* 单阀 PWM 波形循环 (前桥左进气, 示波器看波形)                */
#define TEST_RTE_CAN_PRINT      0u   /* RTE 变量 CAN 打印 (CAN1 0x700, 7 帧轮转, 100ms/帧)         */
#define TEST_PSW_CAN_PRINT      0u   /* PSW 变量 CAN 打印 (CAN1 0x710, 8 帧轮转, 100ms/帧)         */
#define TEST_PRESSURE_CAN       0u   /* 压力/VPOWER 10ms 上报 (CAN1 0x730)                        */
#define TEST_TICK_DEBUG         0u   /* 10ms 节拍验证: 每 10ms 翻转 P22.0 测试点                    */

#endif /* TEST_CFG_H */
