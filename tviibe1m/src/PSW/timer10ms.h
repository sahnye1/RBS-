/**
 * @file    timer10ms.h
 * @brief   10ms 周期定时器 — TCPWM0_GRP0_CNT1 比较模式
 *
 * @details 时钟: clk_peri → 动态分频 → 2MHz → DIVBY_2 → 1MHz, 周期=9999 → 10ms。
 *          ISR 优先级 1 (低于 CAN0/CAN1 的优先级 0): g_systick_ms += 10 (仅调试快照用, 逻辑不依赖) 并置位 g_b10msTick。
 *          (轮速已改 TCPWM+PDMA 硬件搬运, ISR 不再做六通道快照)
 */

#ifndef TIMER10MS_H
#define TIMER10MS_H

#include <stdbool.h>

/* ========================================================================== */
/*  全局标志: 10ms 节拍 (ISR 置 true, 主循环清除)                              */
/* ========================================================================== */
extern volatile bool    g_b10msTick;
extern volatile uint8_t g_timer10msCount;   /* ISR 递增, 供 timer_get_count/timer_count_dev1 */

/* ========================================================================== */
/*  对外 API                                                                   */
/* ========================================================================== */

void Timer10ms_Init(void);

/** @brief 获取 10ms 定时器超时计数, 0=未超时 */
uint8_t timer_get_count(void);

/** @brief 清零 10ms 定时器计数 (g_timer10msCount = 0) */
void timer_count_dev1(void);

/** @brief 记录任务起始时刻 (TCPWM 计数器快照 + 清零溢出计数) */
void rcrd_task_start(void);

/** @brief 计算任务耗时 (单位: 0.1ms), 写入 PSWTaskTime */
void calc_task_time(void);

#endif /* TIMER10MS_H */
