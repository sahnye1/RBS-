/**
 * @file    gpio.h
 * @brief   整车 GPIO 引脚定义 (继电器/UB/低边开关/接插件/报警灯)
 *
 * @note    本文件集中定义所有非 BTS724G 阀控的 GPIO 引脚。
 *          BTS724G 阀控引脚见 bts724g.h。
 *
 *          已确认引脚:
 *            P14.3 → RelayCtrl (总开关)
 *            P13.6 → UBCtrl / UBA_DRV
 *            P18.0 → ABSWLampCtrl / X1_2_staSet / W_LAMP_drv (ABS 故障灯)
 *            P18.4 → HSALampCtrl / HSA_LAMP_in (HSA 坡起灯)
 *            P11.2 → ASRLampCtrl / ASR_LP_drv (ASR 阀故障灯)
 *            P2.0  → ASRFLowSideSw / FarFA_in (前桥 ASR 低边控制)
 *            P6.2  → ASRRLowSideSw / FarDA_in (后桥 ASR 低边控制)
 *            P18.1 → TRABS_SW_IN / X3-4 (挂车 ABS 开关输入)
 *            P11.0 → ESC_SW_IN / X1-6 (ESC 开关输入)
 *            P3.0  → HSA_SW_IN / X1-5 (坡起开关输入)
 */

#ifndef GPIO_H
#define GPIO_H

#include "gpio/cy_gpio.h"
#include "hw_rev.h"

/* ========================================================================== */
/*  继电器控制 (RelayCtrl)                                                      */
/*  P14.3: 总开关, 控制 UB 和 VPOWER 的连接                                     */
/*         打开是 ESC/PRE 供电的前置条件                                         */
/*  GPIO 输出: 0=断开 1=闭合                                                    */
/* ========================================================================== */
#define RELAY_GPIO_PORT         GPIO_PRT14
#define RELAY_GPIO_PIN          3u

/* ========================================================================== */
/*  UB 电压控制 / UBA_DRV (UBCtrl)                                              */
/*  P13.6: 高电平打开 ESC/PRE 供电                                              */
/*  GPIO 输出: 0=关闭 1=开启                                                    */
/* ========================================================================== */
#define UB_GPIO_PORT            GPIO_PRT13
#define UB_GPIO_PIN             6u

/* ========================================================================== */
/*  ABS 故障灯 / W_LAMP_drv (ABSWLampCtrl)                                      */
/*  P18.0: ABS 故障指示灯                                                        */
/*  GPIO 输出: 0=灭 1=亮                                                        */
/* ========================================================================== */
#define ABS_WLAMP_GPIO_PORT     GPIO_PRT18
#define ABS_WLAMP_GPIO_PIN      0u

/* ========================================================================== */
/*  HSA 灯控制 (HSALampCtrl)                                                    */
/*  P18.4: HSA_LAMP_in                                                         */
/*  GPIO 输出: 0=灭 1=亮                                                        */
/* ========================================================================== */
#define HSA_LAMP_GPIO_PORT      GPIO_PRT18
#define HSA_LAMP_GPIO_PIN       4u

/* ========================================================================== */
/*  ASR 阀故障灯控制 (ASRLampCtrl)                                               */
/*  P11.2: ASR_LP_drv                                                          */
/*  GPIO 输出: 0=灭 1=亮                                                        */
/* ========================================================================== */
#define ASR_LAMP_GPIO_PORT      GPIO_PRT11
#define ASR_LAMP_GPIO_PIN       2u

/* ========================================================================== */
/*  接插件开关输入 (Xn_pin_StaGet)                                               */
/* ========================================================================== */

/* X3-4: 挂车 ABS 功能开关 → TRABS_SW_IN (P18.1) */
#ifndef X3_4_GPIO_PORT
#define X3_4_GPIO_PORT          GPIO_PRT18
#endif
#ifndef X3_4_GPIO_PIN
#define X3_4_GPIO_PIN           1u
#endif

/* X1-6: ESC 功能开关 → ESC_SW_IN (P11.0) */
#ifndef X1_6_GPIO_PORT
#define X1_6_GPIO_PORT          GPIO_PRT11
#endif
#ifndef X1_6_GPIO_PIN
#define X1_6_GPIO_PIN           0u
#endif

/* X1-5: 坡起开关 → HSA_SW_IN (P3.0) */
#ifndef X1_5_GPIO_PORT
#define X1_5_GPIO_PORT          GPIO_PRT3
#endif
#ifndef X1_5_GPIO_PIN
#define X1_5_GPIO_PIN           0u
#endif

/* ========================================================================== */
/*  ASR 低边开关 (MOSFET 控制, CK 回读已被其他电路替代)                           */
/* ========================================================================== */

/* 前桥 ASR 低边开关控制 (ASRFLowSideSw) — FarFA_in */
#define ASR_F_LOWSIDE_CTRL_PORT GPIO_PRT2
#define ASR_F_LOWSIDE_CTRL_PIN  0u

/* 前桥 ASR 低边开关状态回读 (ASRFLowSideSt)
 * 与控制脚相同 GPIO, 读回实际电平确认开关状态 */
#define ASR_F_LOWSIDE_STAT_PORT GPIO_PRT2
#define ASR_F_LOWSIDE_STAT_PIN  0u

/* 后桥 ASR 低边开关控制 (ASRRLowSideSw) — FarDA_in */
#define ASR_R_LOWSIDE_CTRL_PORT GPIO_PRT6
#define ASR_R_LOWSIDE_CTRL_PIN  2u

/* 后桥 ASR 低边开关状态回读 (ASRRLowSideSt)
 * 与控制脚相同 GPIO, 读回实际电平确认开关状态 */
#define ASR_R_LOWSIDE_STAT_PORT GPIO_PRT6
#define ASR_R_LOWSIDE_STAT_PIN  2u

/* ========================================================================== */
/*  地码输入 (P8.0): 硬件 ECU 类型编码, 低有效                                  */
/* ========================================================================== */
#define DIMA_GPIO_PORT          GPIO_PRT8
#define DIMA_GPIO_PIN           0u

/* ========================================================================== */
/*  硬件看门狗喂狗 (HardWD_Feed)                                                  */
/*  P5.0: 翻转通过 AC 耦合驱动 QFAR2, 与 QFAR1 串联保持继电器闭合                    */
/* ========================================================================== */
#define WD_HW_GPIO_PORT         GPIO_PRT5
#define WD_HW_GPIO_PIN          0u

/* ========================================================================== */
/*  安全宏: 端口无效时跳过 GPIO 操作, 避免野指针访问                              */
/* ========================================================================== */
#define GPIO_IS_VALID(port, pin)    ((port) != NULL && (pin) < 32u)

/* ========================================================================== */
/*  函数声明                                                                    */
/* ========================================================================== */

/** @brief GPIO 引脚初始化 (继电器/UB/指示灯/接插件输入, 上电默认状态) */
void Gpio_Init(void);

/** @brief 测试计时标记输出 — P22.0 / 电路 LROCN (后桥左排阀, 接插件 X2-5, TCPWM CNT34/LINE34)
 *
 *  @note  测试用例需要在接插件上量到高低电平来测时序 (10ms 节拍 / EEPROM 读写耗时 /
 *         看门狗超时), 故首次调用本函数时把该阀引脚由 TCPWM 切为 GPIO 推挽输出,
 *         之后由本函数直接写电平。切走后该阀不再受 PWM 控制 (TCPWM 照常运行但到不了引脚)。
 *         ⚠ 测试期间这一路会按标记信号通断, 该阀不要接负载; 不切回, 复位后恢复默认。 */
void TestMark_Ctrl(uint8_t state);

/** @brief 读地码 (P8.0): 0=接地 (低有效), 1=悬空/高电平 */
uint8_t Gpio_DiMaRead(void);

/** @brief 继电器控制 (P14.3): 0=断开 1=闭合, 动作后 80ms 自动判定 */
void Gpio_RelayCtrl(uint8_t state);

/** @brief 10ms 节拍推进 (无 pending 时极速返回, 由 PSW_10ms_deal 驱动) */
void Gpio_RelayTick(void);

/** @brief UB 电压控制 (P13.6): 0=关闭 1=开启 */
void Gpio_UBCtrl(uint8_t state);

/** @brief ABS 故障灯 (P18.0): 0=灭 1=亮 */
void Gpio_ABSWLampCtrl(uint8_t state);

/** @brief HSA 坡起灯 (P18.4): 0=灭 1=亮 */
void Gpio_HSALampCtrl(uint8_t state);

/** @brief ASR 阀故障灯 (P11.2): 0=灭 1=亮 */
void Gpio_ASRLampCtrl(uint8_t state);

int8_t Gpio_Xn_pin_StaGet(uint8_t xNum, uint8_t xPin);

/* ========================================================================== */
/*  ASR 低边开关                                                               */
/* ========================================================================== */

/** @brief 前桥 ASR 低边开关控制 (P2.0) */
void     Gpio_ASRFLowSideSw(uint32_t sw);
uint32_t Gpio_ASRFLowSideSt(void);

/** @brief 后桥 ASR 低边开关控制 (P6.2) */
void     Gpio_ASRRLowSideSw(uint8_t sw);
uint32_t Gpio_ASRRLowSideSt(void);

/** @brief 低边开关引用计数使能 (前桥 P2.0 被 22口进(原FA_ASR)+TR_ASR 共享) */
void Gpio_ASRFLowSideEnable(uint8_t enable);
void Gpio_ASRRLowSideEnable(uint8_t enable);

#endif /* GPIO_H */
