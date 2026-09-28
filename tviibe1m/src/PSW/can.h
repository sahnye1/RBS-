#ifndef __CAN_H__
#define __CAN_H__

/**
 * @file    can.h
 * @brief   CAN FD 驱动接口 — Classic CAN 模式
 *
 * @details 本文件含 CAN 驱动接口 + CAN 硬件宏定义 (can.c 可自包含编译),
 *          对外函数签名与 RTE.h 声明一致。
 *          CAN0 = 底盘CAN (CANFD0 Ch1, RX=P0.3 TX=P0.2, J1939)
 *          CAN1 = 私有CAN (CANFD1 Ch0, RX=P14.1 TX=P14.0, 诊断/标定)
 */

#include "cy_project.h"
#include "cy_device_headers.h"

/* ========================================================================== */
/*  CAN 时钟 + 分频器编号                                                      */
/* ========================================================================== */
#define CAN_TARGET_FREQ              40000000ul

/* 外设时钟分频器编号 (div8 分频器, CAN 专用) */
typedef enum
{
    DIV_NO_CAN_CHSS = 0,   /* div8 分频器 0 号 → 底盘CAN */
    DIV_NO_CAN_PRVT = 1,   /* div8 分频器 1 号 → 私有CAN */
    DIV_NO_MAX
} peri_clk_div_no_e;

/* ========================================================================== */
/*  底盘CAN (CANFD0 Ch1)                                                       */
/* ========================================================================== */
#define CAN_CHSS_TYPE                CY_CANFD0_1_TYPE
#define CAN_CHSS_RX_PORT             GPIO_PRT0
#define CAN_CHSS_RX_PIN              3u
#define CAN_CHSS_TX_PORT             GPIO_PRT0
#define CAN_CHSS_TX_PIN              2u
#define CAN_CHSS_RX_MUX              P0_3_CANFD0_TTCAN_RX1
#define CAN_CHSS_TX_MUX              P0_2_CANFD0_TTCAN_TX1
#define CAN_CHSS_PCLK                PCLK_CANFD0_CLOCK_CAN1
#define CAN_CHSS_IRQN                canfd_0_interrupts0_1_IRQn
#define CAN_CHSS_IRQ_INDX            CPUIntIdx1_IRQn

/* ========================================================================== */
/*  私有CAN (CANFD1 Ch0, P14.0=TX P14.1=RX)                                   */
/* ========================================================================== */
#define CAN_PRVT_TYPE                CY_CANFD1_0_TYPE
#define CAN_PRVT_RX_PORT             GPIO_PRT14
#define CAN_PRVT_RX_PIN              1u
#define CAN_PRVT_TX_PORT             GPIO_PRT14
#define CAN_PRVT_TX_PIN              0u
#define CAN_PRVT_RX_MUX              P14_1_CANFD1_TTCAN_RX0
#define CAN_PRVT_TX_MUX              P14_0_CANFD1_TTCAN_TX0
#define CAN_PRVT_PCLK                PCLK_CANFD1_CLOCK_CAN0
#define CAN_PRVT_IRQN                canfd_1_interrupts0_0_IRQn
#define CAN_PRVT_IRQ_INDX            CPUIntIdx2_IRQn

/* ========================================================================== */
/*  CAN 接口 (can0_getMsg/can1_getMsg 声明见 RTE.h, 此处不重复)                 */
/* ========================================================================== */
void can_init(void);
void can_info_cfg(uint8_t canX, uint32_t* canIDList, uint8_t idCount, uint8_t transDirect);
void can0_bitrate_set(uint8_t bitRateFlag);
uint8_t can0_sendMsg(uint32_t msgId, uint8_t *msg, uint8_t msgLen);
uint8_t can1_sendMsg(uint32_t msgId, uint8_t *msg, uint8_t msgLen);
void can0_busoff_resume(void);
void can1_busoff_resume(void);

/* 调试: 接收中断回调计数 (main_cm4.c CAN_TEST 测试用) */
extern volatile uint32_t g_can0_rx_cb_cnt;
extern volatile uint32_t g_can1_rx_cb_cnt;

#endif /* __CAN_H__ */
