/**
 * @file    valve_hold_test.h
 * @brief   21口 / 22口 进气保压测试 — CAN 命令直接驱动 (无状态机)
 *
 * @details 2026-10-14 重构: 原 10ms 非阻塞状态机 (PREP/STEP/FINAL/VENT/DONE)、步进采样、
 *          保压判定、CAN 实时上报全部删除。改为 CAN 命令直接调 RTE 阀函数:
 *            启动 (0x740 data[0]=1): 排气阀 100% 恒高 (通电关住 NO 阀) + 进气阀 PWM 充气
 *            停止 (0x740 data[0]=0): 进气关 + 排气断电 (NO 阀自然排空)
 *          TCPWM 设一次寄存器即持续输出波形, 软件无需 10ms 节拍维护。
 *
 * @details 硬件前提:
 *    - 21口腔压: PSWBrkPreX2_7 (RTEBrkPreX4_14); 22口: PSWBrkPreX3_10 (RTEBrkPreX4_15), kPa
 *    - 进气阀 = 常闭 NC, 通电才进气;  排气阀 = 常开 NO, 必须持续通电才关得住, 断电即排空
 *    - 气源 8atm ≈ 800kPa (传感器量程 0~1050kPa)
 *
 * @details 阀映射 (VHT_CHANNEL 选通道):
 *    0 = 21口 → InValActX3_7 / OutValActX3_10  (U12, 接插件 X3_7 / X3_10)
 *    1 = 22口 → InValActX4_16 / OutValActX2_16  (U13, 接插件 X4_16 / X2_16)
 *
 * @note 低边开关 (P2.0 / P6.2) 由 ASW 侧负责, 本模块不碰。
 * @note 压力监控走通用打印帧 (0x730 每 10ms / 0x700 RTE 打印), 本模块不再自带上报。
 */

#ifndef VALVE_HOLD_TEST_H
#define VALVE_HOLD_TEST_H

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/*  通道选择与参数校验范围                                                     */
/* ========================================================================== */
#define VHT_CHANNEL             0u      /* 0 = 21口 (X3_7/X3_10), 1 = 22口 (X4_16/X2_16) */
#define VHT_CYCLE_MIN_MS        20u     /* 周期下限 (ms) */
#define VHT_CYCLE_MAX_MS        500u    /* 周期上限 (ms; RTE_PERIOD_MAX = 5000 x 0.1ms) */
#define VHT_DUTY_MIN_PCT        1u      /* 占空比下限 */
#define VHT_DUTY_MAX_PCT        90u     /* 占空比上限 (留保压空间) */

/* ========================================================================== */
/*  CAN 命令格式 (CAN1, 标准帧, 上位机 -> ECU)                                   */
/* ========================================================================== */
#define VHT_CAN_CMD_ID          0x740u  /* 命令帧 ID */
/*   B0    cmd     1=启动(充气+保压) / 0=停止(进气关+排气断电排空)              */
/*   B1-B2 cycle   周期 (ms, 小端: B1低 B2高), 10ms 整数倍, 20~500            */
/*   B3    duty    占空比 (%), 1~90;  脉宽 = 周期 x 占空比                     */
/*   B4-B7 保留                                                              */

/**
 * @brief   CAN 命令处理 (无状态机, 直接驱动 RTE 阀函数)
 * @param   cmd        1 = 启动 (排气 100% 恒高 + 进气 PWM 充气)
 *                    0 = 停止 (进气关闭 + 排气断电排空)
 * @param   cycle_ms   周期 (ms), 须 10ms 整数倍且在 20~500 之间 (cmd=0 时忽略)
 * @param   duty_pct   占空比 (%), 1~90 (cmd=0 时忽略)
 * @retval  true  = 命令已执行
 * @retval  false = 参数非法 (未动作, 原状态保持)
 */
bool ValveHoldTest_CanCmd(uint8_t cmd, uint16_t cycle_ms, uint8_t duty_pct);

#endif /* VALVE_HOLD_TEST_H */
