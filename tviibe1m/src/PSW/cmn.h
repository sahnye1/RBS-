/**
 * @file    cmn.h
 * @brief   PSW 公共模块 — 公共宏 + 系统级工具函数
 *
 * @note    GPIO 引脚定义在 gpio.h; 各模块专用配置在各自头文件, 不在此重复。
 */

#ifndef __CMN_H__
#define __CMN_H__

#include "cy_project.h"
#include "cy_device_headers.h"

/* ---- 公共宏 ---- */
#define PI                  (3.141592654f)              /* 圆周率 */
#define DIV_ROUND_UP(a, b)  (((a) + (b) / 2) / (b))     /* 四舍五入整数除法 */

/* ---- SRAM1 起始地址 (system_update_start 用) ---- */
#define RAM_1_ADDRESS       0x08010000

/* ---- 看门狗跳过标记 (PSWdebug 测试用例用) ---- */
extern uint8_t g_skipSoftWD;   /* 1 = 停喂软狗 */
extern uint8_t g_skipHardWD;   /* 1 = 停喂硬狗 */

/* ---- 函数声明 ---- */

/** @brief 外设时钟分频器: 读 clk_peri 实际频率, 按 targetFreq 自动算分频值 (不硬编码源频) */
void periph_divider(en_clk_dst_t ipBlock, cy_en_divider_types_t div_type, uint32_t clk_no, uint32_t targetFreq);

/** @brief 微秒级延时 (SysTick) */
void delay_us(uint32_t n);

/** @brief 软件看门狗初始化 (上限 2000ms) */
void SoftWD_init(void);

/** @brief 双看门狗喂狗: 硬狗 (P5.0 翻转) + 软狗 (Cy_WDT_ClearWatchdog)
 *  @note  wd_feed.h 内有等价的 static inline 版本, 供 PSW 内部模块使用 (同一引脚) */
void wd_feed(void);

/** @brief 读并清除复位原因: 1 = 看门狗复位; 0 = 其他 (仅首次调用有效) */
uint8_t sysRstReason_get(void);

/** @brief 升级入口: 标志写 SRAM1 → NVIC_SystemReset; Bootloader 复位后据此进升级 (不返回) */
void system_update_start(uint32_t updateFlag);

/** @brief 系统软复位 (NVIC_SystemReset) */
void system_reboot(void);

/** @brief 板级初始化: SystemInit + 各模块 Init (含 can_init)
 *  @note  CAN 过滤表须由应用层先调 can_info_cfg() 配置
 *         主循环: for(;;) { if (g_b10msTick) PSW_10ms_deal(); } */
void board_init(void);

/** @brief 10ms 主处理 — 喂狗 → ADC 采集 → PSW* 刷新 → 阀诊断 → 轮速 → 传感器诊断
 *         → 继电器 → PSWDataToRte → CAN busoff 恢复 (逐步顺序见 cmn.c) */
void PSW_10ms_deal(void);

/* 测试/调试入口见 test/test_deal.h (开关在 test/test_cfg.h) */

#endif /* __CMN_H__ */
