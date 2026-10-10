/**
 * @file    rte_psw.h
 * @brief   RTE PSW 层实现 — 驾驶室版 (Cabin Version)
 *
 * @details 实现 RTE.h 中声明的所有 PSW 层函数:
 *          - 17 路阀 PWM 动作封装 (InVal/OutVal/ASRVal → Psw_InValAct/Psw_OutValAct
 *            → ValvePwm_SetDutyLive; 进气阀用 period−htime 起点, 排气/ASR 用周期起点)
 *          - 继电器/UB/低边开关/接插件/CAN/flash/SCC3000 实现
 *          - 阀故障状态同步 (由 psw_data.c:PSWDataToRte 集中处理)
 *          - 报警灯 (HSA/ASR/ABS): 本层 X1_2_staSet/X1_13_staSet/X1_15_staSet 实现,
 *            内部转调 gpio.c 的 Gpio_HSALampCtrl/Gpio_ASRLampCtrl/Gpio_ABSWLampCtrl
 *
 * @note    本文件对应底盘版 PSW 库的角色，驾驶室版自行实现。
 *          以下 P0/P1 为**迁移阶段标注**, 均已完成:
 *          P0: 阀动作封装 (完整实现)
 *          P1: CAN 接口 (已对接 can_driver)
 *          P1: GPIO (继电器/UB/低边开关/接插件/报警灯 — 已对接 gpio.h)
 *          P1: Flash/EEPROM (已对接 eeprom.c)          
 */

#ifndef RTE_PSW_H
#define RTE_PSW_H

/* RTE.h 已声明所有对外函数，本头文件仅作模块内部使用 */

/* 版本号初始化: 给 RTEPSW_Version 赋值 (该变量定义在 rte_psw.c, RTE.c 不重复定义) */
void RtePsw_VersionInit(void);

/* ========================================================================== */
/*  调试: 通信层(ASW→RTE) 四阀 PWM 参数直采 (test_deal 帧14 / 0x71E 用)        */
/*  记录 ASW 最近一次经外包函数 InValActX3_7/OutValActX3_10/InValActX4_16/    */
/*  OutValActX2_16 传入的 period/htime, 判断参数在上游(ASW)是否正常下发。       */
#endif /* RTE_PSW_H */
