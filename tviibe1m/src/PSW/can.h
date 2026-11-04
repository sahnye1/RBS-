#ifndef __CAN_H__
#define __CAN_H__

/**
 * @file    can.h
 * @brief   CAN 通讯驱动接口 — Classic CAN 模式
 *
 * @details CAN0 = 底盘CAN (RBS) / 公共CAN (ABSESC): P0.2=TX P0.3=RX, J1939
 *          CAN1 = 私有CAN (RBS) / 调试CAN (ABSESC): P14.0=TX P14.1=RX, UDS 诊断/标定
 *
 * @note    硬件参数 (引脚/中断/时钟分频) 已下移到 can.c 第 1 节,
 *          本文件只保留对外接口 — 与 gpio.h / adc.h 等外设头文件的组织方式一致。
 *          can0_getMsg() / can1_getMsg() 声明在 RTE.h (跨层门面), 此处不重复。
 *
 * @note    命名: 新增接口统一按 `模块_动作` 风格 (Can_*) 编写;
 *          存量接口 (can_init / can0_sendMsg ...) 的改名涉及 COM/ASW 双侧同步,
 *          留待后续统一处理。
 */
#include <stdint.h>

/* ========================================================================== */
/*  对外接口                                                                    */
/* ========================================================================== */

/** @brief 初始化 CAN0/CAN1 (时钟 + 引脚 + 中断 + SDK 配置)
 *  @note  收发的 ID 白名单须在调用前用 can_info_cfg() 配置, 否则无法收发。 */
void can_init(void);

/** @brief 配置收/发 ID 白名单 (bsw 启动时由应用层下发)
 *  @param canX        0 = CAN0 底盘/公共CAN, 1 = CAN1 私有/调试CAN
 *  @param canIDList   ID 列表 (不得为 NULL)
 *  @param idCount     ID 个数 (最多 128)
 *  @param transDirect 0 = 接收白名单, 1 = 发送白名单
 */
void can_info_cfg(uint8_t canX, uint32_t* canIDList, uint8_t idCount, uint8_t transDirect);

/** @brief CAN0 波特率设置: 0=1M / 1=500k / 2=250k / 3=125k / 其他=500k */
void can0_bitrate_set(uint8_t bitRateFlag);

/** @brief CAN0 发送 (ID 须在发送白名单内)
 *  @param msgId  报文 ID (≤0x7FF 为标准帧, 否则扩展帧)
 *  @param msg    数据 (不足 8 字节由 0xFF 填充)
 *  @param msgLen 有效数据长度 (1~8)
 *  @return 0 = 成功; 1 = 参数错误或 ID 不在发送白名单; 2 = 发送缓冲/HW 失败
 */
uint8_t can0_sendMsg(uint32_t msgId, uint8_t *msg, uint8_t msgLen);

/** @brief CAN1 发送 (不做白名单检查, 轮询空闲 TX 缓冲)
 *  @param msgId  报文 ID (≤0x7FF 为标准帧, 否则扩展帧)
 *  @param msg    数据 (不足 8 字节由 0xFF 填充)
 *  @param msgLen 有效数据长度 (1~8)
 *  @return 0 = 成功; 1 = 参数错误或 32 个 TX 缓冲全忙; 2 = 发送缓冲/HW 失败
 */
uint8_t can1_sendMsg(uint32_t msgId, uint8_t *msg, uint8_t msgLen);

/** @brief CAN0 busoff 恢复节拍推进 (每 10ms 调用一次) */
void can0_busoff_resume(void);

/** @brief CAN1 busoff 恢复节拍推进 (每 10ms 调用一次) */
void can1_busoff_resume(void);

/** @brief 读 busoff 中断状态捕获值 (中断里锁存的 M_TTCAN.unIR)
 *  @param canX 0 = CAN0, 1 = CAN1
 *  @return 状态寄存器原值, 0 = 无 busoff 事件
 *  @note   取代外部手写 extern 直接访问内部变量; 需要自定义掩码判断时用本返回值。
 */
uint32_t Can_GetBusoffFlag(uint8_t canX);

/** @brief 读发送白名单条数 (由 can_info_cfg 配置)
 *  @param canX 0 = CAN0, 1 = CAN1
 *  @note   CAN1 当前不做发送过滤, 条数仅作记录/调试查询。
 */
uint8_t Can_GetSendWhitelistCount(uint8_t canX);

/* 调试: 接收中断回调计数 (main_cm4.c CAN_TEST 测试用) */
extern volatile uint32_t g_can0_rx_cb_cnt;
extern volatile uint32_t g_can1_rx_cb_cnt;

#endif /* __CAN_H__ */
