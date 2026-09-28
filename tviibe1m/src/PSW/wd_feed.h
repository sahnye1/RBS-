/**
 * @file    wd_feed.h
 * @brief   看门狗喂狗 — 软狗 (WDT) + 硬狗 (GPIO 翻转)
 *
 * @note    独立于 PSW 层, PSW 层自用。硬狗引脚 P5.0。
 *          ⚠ 与 cmn.c 的 wd_feed() 同名 (cmn.c 提供非 inline 版本给外部),
 *            同一编译单元不要同时包含 cmn.h 与本文件。
 */

#ifndef WD_FEED_H
#define WD_FEED_H

#include "cy_project.h"
#include "cy_device_headers.h"

#define WD_HW_PORT  GPIO_PRT5
#define WD_HW_PIN   0u

static inline void wd_feed(void)
{
    Cy_GPIO_Inv(WD_HW_PORT, WD_HW_PIN);   /* 硬件看门狗 */
    Cy_WDT_ClearWatchdog();               /* 软件看门狗 */
}

#endif /* WD_FEED_H */
