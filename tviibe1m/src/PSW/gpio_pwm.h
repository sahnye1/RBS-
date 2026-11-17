/**
 * @file    gpio_pwm.h
 * @brief   GPIO 软件 PWM — ABS 阀 (12 路), 100μs 定时器中断驱动
 *
 * @details ABS 阀 (6 轮制动阀, 通电即打开) 改用 GPIO 位翻转模拟 PWM,
 *          替代 TCPWM 硬件 PWM (因该阀电气特性与 ASR/ABS 电磁阀不同)。
 *
 *          时基 100μs: 周期 10ms 时占空比分辨率 100 级 (1%)。
 *          参数语义与硬件 PWM 一致 (period / cc0_start / htime, 单位 0.1ms)。
 *          边沿由 100μs 中断比较 counter 产生, 抖动 ±100μs (受中断抢占影响)。
 *
 * @note    由 valve_pwm.h 的 VALVE_CTRL_GPIO_MODE 控制:
 *            0 = 不编译 (默认, ABS 阀也走 TCPWM 硬件 PWM)
 *            1 = 编译, ABS 阀走 GPIO 软件 PWM (21/22 口仍走 TCPWM)
 *          切换只需改该宏后重新编译, 上层 (rte_psw.c/bts724g.c) 不用改。
 */

#ifndef GPIO_PWM_H
#define GPIO_PWM_H

#include <stdint.h>
#include <stdbool.h>
#include "bts724g.h"     /* valve_id_t */
#include "valve_pwm.h"   /* VALVE_CTRL_GPIO_MODE */

#if (VALVE_CTRL_GPIO_MODE != 0u)

/* ========================================================================== */
/*  通道数 (ABS 阀 = 6 轮制动 × 进/排)                                          */
/* ========================================================================== */
#define GPIO_PWM_CH_NUM   12u

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

/** @brief GPIO 软件 PWM 初始化 (100μs 定时器 + 中断挂载)
 *  @note  由 cmn.c board_init 里 Bts724g_Init() 之后调用
 */
void GpioPwm_Init(void);

/** @brief 设置 PWM 参数 (0.1ms 单位, 与硬件 PWM 的 RTE 语义一致)
 *  @param vid       阀 ID (仅 ABS 阀; 非 ABS 阀调用无操作)
 *  @param period    周期 (0.1ms), 0 = 非法忽略
 *  @param cc0_start 高电平起点 (0.1ms)
 *  @param htime     高电平宽度 (0.1ms)
 */
void GpioPwm_SetDuty(valve_id_t vid, uint16_t period, uint16_t cc0_start, uint16_t htime);

/** @brief 直接开/关 (诊断/上电直写用, 会挂起软件 PWM)
 *  @param vid 阀 ID (仅 ABS 阀)
 *  @param on  true=高电平(阀开), false=低电平(阀关)
 */
void GpioPwm_SetOnOff(valve_id_t vid, bool on);

/** @brief 100μs 定时器中断向量入口 (仅供中断注册, 应用层不要调用) */
void GpioPwm_ISR(void);

#endif /* VALVE_CTRL_GPIO_MODE */

#endif /* GPIO_PWM_H */
