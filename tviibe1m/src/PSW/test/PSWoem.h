/**
 * @file    PSWoem.h
 * @brief   OEM 模式动作框架 — 声明 (实现见 PSWoem.c)
 *
 * @note    每 10ms 调用一次 (cmn.c 的 PSW_10ms_deal ⑧.6 步):
 *          模式判定: RTEfTestMode == PSWOEM_MODE_OEM(2) 时框架生效。
 *          命令通道 (暂用现有调试变量, 通讯层确定正式变量后只改 PSWoem.c 的读接口):
 *            RTEfDbgMsgSW  = 命令字 (执行后置 0xFF; 与 PSWdebug 用的 SW3 不冲突)
 *            RTEfDbgMsgSW1 = 参数1 (命令相关, 例如 valve_id)
 *            RTEfDbgMsgSW2 = 参数2 (命令相关, 预留)
 *          命令表见 PSWoem.c 的 s_oem_cmd_list[], 新增命令 = 加一条表项 + 一个处理函数。
 *
 * @note    变量替换: 通讯层若新增 OEM 专用变量 (例如命令/参数分开的专用接口),
 *          只需改 PSWoem.c 第 3 节的 oem_mode_get() / oem_cmd_get() / oem_cmd_arg1() /
 *          oem_cmd_arg2() / oem_cmd_ack() 这 5 个函数; 命令表与阀动作不用动。
 *          RTE.h / RTE.c 的变量定义属三层共享接口, 新增须与 COM/ASW 双边确认后再动。
 */
#ifndef PSWOEM_H
#define PSWOEM_H

/* ========================================================================== */
/*  模式值 (与 RTEfTestMode 的取值定义一致)                                     */
/* ========================================================================== */
#define PSWOEM_MODE_NORMAL      0u    /* 正常模式 — 框架不动作              */
#define PSWOEM_MODE_FACTORY     1u    /* 工厂测试模式 — 框架不动作          */
#define PSWOEM_MODE_OEM         2u    /* OEM 模式 — 框架接收命令并执行动作  */

/* ========================================================================== */
/*  命令字 (RTEfDbgMsgSW 的取值)                                                */
/* ========================================================================== */
#define PSWOEM_CMD_NONE         0xFFu /* 空闲/无命令 (执行完由本模块置回)   */
#define PSWOEM_CMD_ALL_OFF      0x00u /* 关闭全部阀 — p1/p2 未用            */
#define PSWOEM_CMD_VALVE_ON     0x01u /* 打开指定阀 — p1 = valve_id (0~15)  */
#define PSWOEM_CMD_VALVE_OFF    0x02u /* 关闭指定阀 — p1 = valve_id (0~15)  */

/* ========================================================================== */
/*  对外接口                                                                    */
/* ========================================================================== */

/** @brief OEM 模式动作框架入口 (每 10ms 调用一次) */
void PSWoem(void);

#endif /* PSWOEM_H */
