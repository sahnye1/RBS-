/**
 * @file    can.c
 * @brief   CAN 通讯驱动 — CAN0/CAN1 (Classic CAN, 500kbps) + 接收中断 + busoff 恢复
 *
 * @details 两个实例 (引脚/中断宏定义见本文件第 1 节):
 *            CAN0 = CAN_CHSS (RBS 称"底盘CAN" / ABSESC 称"公共CAN"; J1939): P0.2=TX(P0_2_CANFD0_TTCAN_TX1) P0.3=RX,
 *                   IRQ canfd_0_interrupts0_1_IRQn → CPUIntIdx1_IRQn (优先级 0)
 *            CAN1 = CAN_PRVT (RBS 称"私有CAN" / ABSESC 称"调试CAN"; UDS 诊断/标定): P14.0=TX(P14_0_CANFD1_TTCAN_TX0) P14.1=RX,
 *                   IRQ canfd_1_interrupts0_0_IRQn → CPUIntIdx2_IRQn (优先级 0)
 *          时钟 40MHz, prescaler=10 → 4MHz, 1bit=8tq → **500kbps** (Classic CAN, 数据场 8 字节)
 *          收发白名单: 由应用层 can_info_cfg() 配置 (不在白名单内的 ID 不收/不发)
 *          接收: 中断回调写入 g_canX_recvMsg[] 缓冲, 应用用 canX_getMsg() 批量取走
 *
 * @note    ⚠ busoff 恢复依赖 **u1BOE 中断**: SDK 的 Cy_CANFD_Init() 只使能 RX 中断
 *          (DRXE/RF0NE/RF1NE) 且每次调用都重写 unIE → Bus_Off 状态中断恒为 0 →
 *          进 busoff 后已无 RX 事件 → 永远恢复不了。
 *          修复: 在 4 处 Cy_CANFD_Init() 之后补
 *                CAN_xxx_TYPE->M_TTCAN.unIE.stcField.u1BOE = 1u;
 *          (can_init 2 处 + busoff_resume 重 init 2 处; 只开 BO, 不开 EW/EP/BEUE)
 *          2026-09-22 台架验证通过。
 */

#include "RTE.h"
#include "can.h"
#include "cmn.h"
#include <string.h>

/* ========================================================================== */
/*  1. 硬件参数 (时钟 / 引脚 / 中断)                                            */
/*                                                                            */
/*  仅本文件使用 — 原定义在 can.h, 已下移, 使上层 include can.h 时不再被拖着      */
/*  引入整套芯片头 (cy_project.h / cy_device_headers.h)。                        */
/* ========================================================================== */
#define CAN_TARGET_FREQ              40000000ul

/* 外设时钟分频器编号 (div8 分频器, CAN 专用) */
typedef enum
{
    DIV_NO_CAN_CHSS = 0,   /* div8 分频器 0 号 → 底盘CAN */
    DIV_NO_CAN_PRVT = 1,   /* div8 分频器 1 号 → 私有CAN */
    DIV_NO_MAX
} peri_clk_div_no_e;

/* 底盘CAN (CANFD0 Ch1): P0.2=TX P0.3=RX */
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

/* 私有CAN (CANFD1 Ch0): P14.0=TX P14.1=RX */
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
/*  2. 全局状态                                                                 */
/*                                                                            */
/*  收发白名单 / 接收缓冲: 保持非 static (调试时可外部观察);                     */
/*  计数与 busoff 状态机: 内部状态, static 化, 对外经 Can_GetBusoffFlag() 读取。   */
/* ========================================================================== */
uint32_t g_can0_sendID_lst[32] = {0};
static uint8_t s_can0_snd_msgId_count;

uint32_t g_can1_sendID_lst[32] = {0};
static uint8_t s_can1_snd_msgId_count;

Can_recv_msg_st g_can0_recvMsg[32];
static uint8_t s_can0_rcv_msgId_count;

Can_recv_msg_st g_can1_recvMsg[32];
static uint8_t s_can1_rcv_msgId_count;

//底盘CAN标准ID过滤配置
static cy_stc_id_filter_t g_can0_stdIdFilter[32];

//底盘CAN扩展ID过滤配置
static cy_stc_extid_filter_t g_can0_extIdFilter[32];


//私有CAN标准ID过滤配置
static cy_stc_id_filter_t g_can1_stdIdFilter[32];

//私有CAN扩展ID过滤配置
static cy_stc_extid_filter_t g_can1_extIdFilter[32];


/* ========================================================================== */
/*  3. 硬件配置 (SDK 配置 / 引脚 / 中断向量)                                    */
/* ========================================================================== */

/** 底盘CAN 全局配置 (canFDMode=false → Classic CAN, 500kbps, 采样点 75%) */
cy_stc_canfd_config_t canCfg = 
{
	.txCallback     = NULL, // Unused.
	.rxCallback     = NULL,
	.rxFifoWithTopCallback = NULL,//CAN_RxFifoWithTopCallback,
	.statusCallback = NULL, // Un-supported now
	.errorCallback  = NULL, // Un-supported now
	.canFDMode      = false, // Use standard CAN mode

  // 40 MHz   // 500KHz
	.bitrate        =       // Nominal bit rate settings (sampling point = 75%)
	{
		.prescaler      = 10u - 1u,  // cclk/10, When using 500kbps, 1bit = 8tq
		.timeSegment1   = 5u - 1u, // tseg1 = 5tq
		.timeSegment2   = 2u - 1u,  // tseg2 = 2tq
		.syncJumpWidth  = 2u - 1u,  // sjw   = 2tq
	},
	.globalFilterConfig =   // Global filter
	{
		.nonMatchingFramesStandard = CY_CANFD_REJECT_NON_MATCHING,  // Reject none match IDs
		.nonMatchingFramesExtended = CY_CANFD_REJECT_NON_MATCHING,  // Reject none match IDs
		.rejectRemoteFramesStandard = true, // No remote frame
		.rejectRemoteFramesExtended = true, // No remote frame
	},
	.rxBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
	.txBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
	.noOfRxBuffers = 64u,
	.noOfTxBuffers = 32u,
};

/** 私有CAN 全局配置 (与 canCfg 相同, 单独一份便于独立改波特率) */
cy_stc_canfd_config_t canCfg1 = 
{
	.txCallback     = NULL, // Unused.
	.rxCallback     = NULL,
	.rxFifoWithTopCallback = NULL,//CAN_RxFifoWithTopCallback,
	.statusCallback = NULL, // Un-supported now
	.errorCallback  = NULL, // Un-supported now
	.canFDMode      = false, // Use standard CAN mode

     // 40 MHz   500KHz
	.bitrate        =       // Nominal bit rate settings (sampling point = 80%)
	{
		.prescaler      = 4u - 1u,  // 40MHz/4 = 10MHz, 1tq = 100ns
		.timeSegment1   = 15u - 1u, // tseg1 = 15tq
		.timeSegment2   = 4u - 1u,  // tseg2 = 4tq
		.syncJumpWidth  = 4u - 1u,  // sjw   = 4tq (= tseg2 上限)
	},
	.globalFilterConfig =   // Global filter
	{
		.nonMatchingFramesStandard = CY_CANFD_REJECT_NON_MATCHING,  // Reject none match IDs
		.nonMatchingFramesExtended = CY_CANFD_REJECT_NON_MATCHING,  // Reject none match IDs
		.rejectRemoteFramesStandard = true, // No remote frame
		.rejectRemoteFramesExtended = true, // No remote frame
	},
	.rxBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
	.txBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
	.noOfRxBuffers = 64u,
	.noOfTxBuffers = 32u,
};


/** 引脚配置: CAN0 RX/TX + CAN1 RX/TX (顺序与下方表项一一对应) */
static const cy_stc_gpio_pin_prt_config_t g_can_pin_cfg[] =
{
//       portReg,             pinNum,  outVal,   driveMode,         hsiom,      intEdge,intMask,vtrip,slewRate,driveSel,vregEn,ibufMode,vtripSel,vrefSel,vohSel
    { CAN_CHSS_RX_PORT, CAN_CHSS_RX_PIN, 0ul, CY_GPIO_DM_HIGHZ,  CAN_CHSS_RX_MUX, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul}, /* CAN0_2 RX -- CHASSIS RX*/
    { CAN_CHSS_TX_PORT, CAN_CHSS_TX_PIN, 1ul, CY_GPIO_DM_STRONG, CAN_CHSS_TX_MUX, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul}, /* CAN0_2 TX -- CHASSIS TX*/
    { CAN_PRVT_RX_PORT, CAN_PRVT_RX_PIN, 0ul, CY_GPIO_DM_HIGHZ,  CAN_PRVT_RX_MUX, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul}, /* CAN0_2 RX -- PRIVATE RX*/
    { CAN_PRVT_TX_PORT, CAN_PRVT_TX_PIN, 1ul, CY_GPIO_DM_STRONG, CAN_PRVT_TX_MUX, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul}, /* CAN0_2 TX -- PRIVATE TX*/

};
#define CAN_PORT_NUM (sizeof(g_can_pin_cfg)/sizeof(g_can_pin_cfg[0]))

/** 底盘CAN 接收中断配置 */
cy_stc_sysint_irq_t g_can0_irq_cfg = {
	.sysIntSrc = CAN_CHSS_IRQN,
	.intIdx = CAN_CHSS_IRQ_INDX,
	.isEnabled = true,
};

/** 私有CAN 接收中断配置 */
cy_stc_sysint_irq_t g_can1_irq_cfg = {
	.sysIntSrc = CAN_PRVT_IRQN,
	.intIdx = CAN_PRVT_IRQ_INDX,
	.isEnabled = true,
};

/* 调试: 接收中断回调计数 (main_cm4.c CAN_TEST 测试用) */
volatile uint32_t g_can0_rx_cb_cnt = 0u;
volatile uint32_t g_can1_rx_cb_cnt = 0u;

/* ========================================================================== */
/*  4. 接收回调 (中断上下文)                                                     */
/*                                                                            */
/*  按 ID 在白名单缓冲里定位槽位 → 拆 8 字节小端数据 → 置接收标志;               */
/*  应用侧用 canX_getMsg() 取走并清标志。                                        */
/* ========================================================================== */

/** @brief CAN0 接收回调: 按 ID 匹配 g_can0_recvMsg[] 槽位并填充数据
 *  @note  中断上下文; 非 8 字节报文直接丢弃 (本工程只处理 Classic CAN 8 字节)。
 */
static void can_rxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_msg_t* pstcCanFDmsg)
{
	uint8_t i,j;
	uint8_t byte4Count = 0;

	g_can0_rx_cb_cnt++;

	if (pstcCanFDmsg->dataConfig.dataLengthCode != 8)
	{
		return;
	}

	for (i = 0; i < s_can0_rcv_msgId_count; i++)
	{
		if (pstcCanFDmsg->idConfig.identifier == g_can0_recvMsg[i].msgID)
		{
			break;
		}
	}

	//if (i == g_can_msgId_count || g_can_recvMsg[i].msgRcvFlag == true)  //旧数据未处理，新数据丢掉
	if (i == s_can0_rcv_msgId_count)
	{
		return;
	}

	for(j = 0; j < pstcCanFDmsg->dataConfig.dataLengthCode; j++)
	{
		byte4Count = j < 4 ? 0 : 1;
		g_can0_recvMsg[i].msg[j] = (uint8_t)(pstcCanFDmsg->dataConfig.data[byte4Count] >> ((j % 4) * 8));
	}
	g_can0_recvMsg[i].msgRcvFlag = 1;
}

/** @brief CAN1 接收回调: 按 ID 匹配 g_can1_recvMsg[] 槽位并填充数据
 *  @note  中断上下文; 非 8 字节报文直接丢弃。
 */
static void can1_rxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_msg_t* pstcCanFDmsg)
{
	uint8_t i,j;
	uint8_t byte4Count = 0;

	g_can1_rx_cb_cnt++;

	if (pstcCanFDmsg->dataConfig.dataLengthCode != 8)
	{
		return;
	}

	for (i = 0; i < s_can1_rcv_msgId_count; i++)
	{
		if (pstcCanFDmsg->idConfig.identifier == g_can1_recvMsg[i].msgID)
		{
			break;
		}
	}

	//if (i == g_can_msgId_count || g_can_recvMsg[i].msgRcvFlag == true)  //旧数据未处理，新数据丢掉
	if (i == s_can1_rcv_msgId_count)
	{
		return;
	}

	for(j = 0; j < pstcCanFDmsg->dataConfig.dataLengthCode; j++)
	{
		byte4Count = j < 4 ? 0 : 1;
		g_can1_recvMsg[i].msg[j] = (uint8_t)(pstcCanFDmsg->dataConfig.data[byte4Count] >> ((j % 4) * 8));
	}
	g_can1_recvMsg[i].msgRcvFlag = 1;
}

/* ========================================================================== */
/*  5. busoff 恢复                                                              */
/*                                                                            */
/*  中断里把 unIR 锁存到 s_busoff_flagX, 主循环每 10ms 调一次 resume:            */
/*  检到 BO 事件 → 等 s_max_resume_timeX 拍 → 重 init 一次;                      */
/*  连续重试 10 次后把等待拉到 100 拍 (总线持续故障时不频繁重 init)。              */
/*                                                                            */
/*  ⚠ 每次重 init 之后必须补开 u1BOE, 否则下次 busoff 不进中断 → 永远恢复不了。   */
/* ========================================================================== */

/* 底盘CAN busoff 状态机 (内部状态) */
static uint32_t s_busoff_flag0 = 0;
static uint8_t  s_resume_count0 = 0;
static uint8_t  s_resume_time_ms0 = 0;
static uint8_t  s_max_resume_time0 = 5;

/** @brief CAN0 busoff 恢复节拍 (每 10ms 调用一次)
 *  @note  掩码 0x3E00000 = BO/EP/EW 错误状态位; 恢复逻辑 2026-09-22 台架验证通过, 勿改。
 */
void can0_busoff_resume(void)
{
	//if ((s_busoff_flag0 & 0x3ff7E0EE) > 0)
	//if ((s_busoff_flag0 & 0x2000000) > 0)
	if ((s_busoff_flag0 & 0x3E00000) > 0)
	{
		if (s_resume_count0 >= 10)
		{
			//return;
			s_max_resume_time0 = 100;
		}
		if (s_resume_time_ms0 < s_max_resume_time0)
		{
			s_resume_time_ms0++;
		}
		else
		{
			s_resume_time_ms0=0;
			if (s_resume_count0 < 10)
				s_resume_count0++;
			//uart_printf("resume time = %d, resume_count = %d\r\n", resume_time_ms, resume_count);
			//恢复一次
			canCfg.rxCallback = can_rxMsgCallback;
			canCfg.sidFilterConfig.numberOfSIDFilters = sizeof(g_can0_stdIdFilter) / sizeof(cy_stc_id_filter_t);
			canCfg.sidFilterConfig.sidFilter = g_can0_stdIdFilter;
			canCfg.extidFilterConfig.numberOfEXTIDFilters = sizeof(g_can0_extIdFilter) / sizeof(cy_stc_extid_filter_t);
			canCfg.extidFilterConfig.extidFilter = g_can0_extIdFilter;
			canCfg.extidFilterConfig.extIDANDMask = 0x1fffffff;
			Cy_CANFD_Init(CAN_CHSS_TYPE, &canCfg);
			/* 补开 Bus_Off 状态中断: SDK 的 Cy_CANFD_Init 只使能 RX 中断 (DRXE/RF0NE/RF1NE), 且每次调用都会重写 unIE
			 * 不开此位则 busoff 时无中断 → s_busoff_flag0 恒为 0 → 恢复函数永不动作 */
			CAN_CHSS_TYPE->M_TTCAN.unIE.stcField.u1BOE = 1u;

			s_busoff_flag0 = 0;
		}
	}
	else
	{
		s_resume_count0 = 0;
		s_resume_time_ms0 = 0;
		s_max_resume_time0 = 5;
	}
}


/** @brief CAN0 中断入口: 锁存 unIR → 交给 SDK 处理 → 写回清标志 */
static void can0_interruptHandler(void)
{
	s_busoff_flag0 = CAN_CHSS_TYPE->M_TTCAN.unIR.u32Register;
	//uart_printf("int handler %x\r\n", s_busoff_flag0);
	Cy_CANFD_IrqHandler(CAN_CHSS_TYPE);
	CAN_CHSS_TYPE->M_TTCAN.unIR.u32Register = s_busoff_flag0;
}

/* 私有CAN busoff 状态机 (内部状态) */
static uint32_t s_busoff_flag1 = 0;
static uint8_t  s_resume_count1 = 0;
static uint8_t  s_resume_time_ms1 = 0;
static uint8_t  s_max_resume_time1 = 5;

/** @brief CAN1 busoff 恢复节拍 (每 10ms 调用一次)
 *  @note  逻辑与 CAN0 相同 (掩码 0x3E00000); 恢复逻辑 2026-09-22 台架验证通过, 勿改。
 */
void can1_busoff_resume(void)
{
	//if ((s_busoff_flag1 & 0x2000000) > 0)
	if ((s_busoff_flag1 & 0x3E00000) > 0)
	//if ((s_busoff_flag1 & 0x3ff7E0EE) > 0)
	{
		if (s_resume_count1 >= 10)
		{
			//return;
			s_max_resume_time1 = 100;
		}
		if (s_resume_time_ms1 < s_max_resume_time1)
		{
			s_resume_time_ms1++;
		}
		else
		{
			s_resume_time_ms1=0;
			if (s_resume_count1 < 10)
				s_resume_count1++;
			//uart_printf("resume time = %d, resume_count = %d\r\n", resume_time_ms, resume_count);
			//恢复一次
			canCfg1.rxCallback = can1_rxMsgCallback;
			canCfg1.sidFilterConfig.numberOfSIDFilters = sizeof(g_can1_stdIdFilter) / sizeof(cy_stc_id_filter_t);
			canCfg1.sidFilterConfig.sidFilter = g_can1_stdIdFilter;
			canCfg1.extidFilterConfig.numberOfEXTIDFilters = sizeof(g_can1_extIdFilter) / sizeof(cy_stc_extid_filter_t);
			canCfg1.extidFilterConfig.extidFilter = g_can1_extIdFilter;
			canCfg1.extidFilterConfig.extIDANDMask = 0x1fffffff;
			Cy_CANFD_Init(CAN_PRVT_TYPE, &canCfg1);
			/* 补开 Bus_Off 状态中断: SDK 的 Cy_CANFD_Init 只使能 RX 中断 (DRXE/RF0NE/RF1NE), 且每次调用都会重写 unIE
			 * 不开此位则 busoff 时无中断 → s_busoff_flag0 恒为 0 → 恢复函数永不动作 */
			CAN_PRVT_TYPE->M_TTCAN.unIE.stcField.u1BOE = 1u;

			s_busoff_flag1 = 0;
		}
	}
	else
	{
		s_resume_count1 = 0;
		s_resume_time_ms1 = 0;
		s_max_resume_time1 = 5;
	}
}



/** @brief CAN1 中断入口: 锁存 unIR → 交给 SDK 处理 → 写回清标志 */
static void can1_interruptHandler(void)
{
	s_busoff_flag1 = CAN_PRVT_TYPE->M_TTCAN.unIR.u32Register;
	//uart_printf("int handler %x\r\n", s_busoff_flag1);
	Cy_CANFD_IrqHandler(CAN_PRVT_TYPE);
	CAN_PRVT_TYPE->M_TTCAN.unIR.u32Register = s_busoff_flag1;
}

/* ========================================================================== */
/*  6. 收发白名单配置                                                           */
/* ========================================================================== */

/** @brief 配置收/发 ID 白名单 (启动时由应用层下发)
 *  @param canX        0 = CAN0 底盘/公共CAN, 1 = CAN1 私有/调试CAN
 *  @param canIDList   ID 列表 (NULL 或 idCount=0 时直接返回)
 *  @param idCount     ID 个数 (上限 128)
 *  @param transDirect 0 = 接收白名单, 1 = 发送白名单
 */
void can_info_cfg(uint8_t canX, uint32_t* canIDList, uint8_t idCount, uint8_t transDirect)
{
	if (canIDList == NULL || idCount == 0)
	{
		return;
	}

	if ((canX != 0 && canX != 1) || transDirect > 1)
	{
		return;
	}
	uint8_t classCnt = 0;   //标准ID数
	uint8_t extIdCnt = 0;   //扩展ID数
	
	if (canX == 0)   //底盘CAN
	{
		if (transDirect == 0)   //接收BUF
		{
			memset(g_can0_stdIdFilter, 0x00, sizeof(g_can0_stdIdFilter));
			memset(g_can0_extIdFilter, 0x00, sizeof(g_can0_extIdFilter));
			memset(g_can0_recvMsg, 0x00, sizeof(g_can0_recvMsg));
			s_can0_rcv_msgId_count = 0;
			for (uint8_t i = 0; i < idCount; i++)
			{
				if (canIDList[i] > 0x7FFFFFFF)
				{
					continue;
				}
				else if (canIDList[i] > 0x7FF)
				{
					g_can0_extIdFilter[extIdCnt].f0_f.efid1  = (uint32_t)(canIDList[i]);
					g_can0_extIdFilter[extIdCnt].f0_f.efec   = (uint32_t)CY_CANFD_ID_FILTER_ELEMNT_CONFIG_STORE_RXBUFF_OR_DEBUGMSG;
					g_can0_extIdFilter[extIdCnt].f1_f.efid2  = (0u << 9u) | (uint32_t)(extIdCnt);
					g_can0_extIdFilter[extIdCnt].f1_f.eft    = (uint32_t)CY_CANFD_EXT_ID_FILTER_TYPE_CLASSIC;
					extIdCnt++;
					
				}
				else if (canIDList[i] <= 0x7FF)
				{
					g_can0_stdIdFilter[classCnt].sfid2 = (0u << 9u) | (uint32_t)(classCnt);
			        g_can0_stdIdFilter[classCnt].sfid1 = (uint32_t)(canIDList[i]);
			        g_can0_stdIdFilter[classCnt].sfec  = (uint32_t)CY_CANFD_ID_FILTER_ELEMNT_CONFIG_STORE_RXBUFF_OR_DEBUGMSG;
			        g_can0_stdIdFilter[classCnt].sft   = (uint32_t)CY_CANFD_STD_ID_FILTER_TYPE_CLASSIC;
					classCnt++;
				}
				g_can0_recvMsg[s_can0_rcv_msgId_count].msgID = canIDList[i];
				g_can0_recvMsg[s_can0_rcv_msgId_count].msgRcvFlag = 0;
				s_can0_rcv_msgId_count++;
			}
		}
		else   //发送BUF
		{
			memset(g_can0_sendID_lst, 0x00, sizeof(g_can0_sendID_lst));
			if (idCount > 32)
			{
				memcpy(g_can0_sendID_lst, canIDList, 32*sizeof(g_can0_sendID_lst[0]));
				s_can0_snd_msgId_count = 32;
			}
			else
			{
				memcpy(g_can0_sendID_lst, canIDList, idCount*sizeof(g_can0_sendID_lst[0]));
				s_can0_snd_msgId_count = idCount;
			}
		}
	}
	else  //私有CAN
	{
		if (transDirect == 0)  //接收BUF
		{
			memset(g_can1_stdIdFilter, 0x00, sizeof(g_can1_stdIdFilter));
			memset(g_can1_extIdFilter, 0x00, sizeof(g_can1_extIdFilter));
			memset(g_can1_recvMsg, 0x00, sizeof(g_can1_recvMsg));
			s_can1_rcv_msgId_count = 0;
			for (uint8_t i = 0; i < idCount; i++)
			{
				if (canIDList[i] > 0x7FFFFFFF)
				{
					continue;
				}
				else if (canIDList[i] > 0x7FF)
				{
					g_can1_extIdFilter[extIdCnt].f0_f.efid1  = (uint32_t)canIDList[i];
					g_can1_extIdFilter[extIdCnt].f0_f.efec   = (uint32_t)CY_CANFD_ID_FILTER_ELEMNT_CONFIG_STORE_RXBUFF_OR_DEBUGMSG;
					g_can1_extIdFilter[extIdCnt].f1_f.efid2  = (0u << 9u) | (uint32_t)(extIdCnt);
					g_can1_extIdFilter[extIdCnt].f1_f.eft    = (uint32_t)CY_CANFD_EXT_ID_FILTER_TYPE_CLASSIC;
					extIdCnt++;
				}
				else if (canIDList[i] <= 0x7FF)
				{
					g_can1_stdIdFilter[classCnt].sfid2 = (0u << 9u) | (uint32_t)(classCnt);
			        g_can1_stdIdFilter[classCnt].sfid1 = (uint32_t)(canIDList[i]);
			        g_can1_stdIdFilter[classCnt].sfec  = (uint32_t)CY_CANFD_ID_FILTER_ELEMNT_CONFIG_STORE_RXBUFF_OR_DEBUGMSG;
			        g_can1_stdIdFilter[classCnt].sft   = (uint32_t)CY_CANFD_STD_ID_FILTER_TYPE_CLASSIC;
					classCnt++;
				}
				g_can1_recvMsg[s_can1_rcv_msgId_count].msgID = canIDList[i];
				g_can1_recvMsg[s_can1_rcv_msgId_count].msgRcvFlag = 0;
				s_can1_rcv_msgId_count++;
			}
		}
		else    //发送BUF
		{
			memset(g_can1_sendID_lst, 0x00, sizeof(g_can1_sendID_lst));
			if (idCount > 128)
			{
				memcpy(g_can1_sendID_lst, canIDList, 128*sizeof(g_can1_sendID_lst[0]));
				s_can1_snd_msgId_count = 128;
			}
			else
			{
				memcpy(g_can1_sendID_lst, canIDList, idCount*sizeof(g_can1_sendID_lst[0]));
				s_can1_snd_msgId_count = idCount;
			}
		}
	}
}

/* ========================================================================== */
/*  7. 收发接口                                                                 */
/* ========================================================================== */

/** @brief CAN0 批量取接收报文 (取走即清接收标志)
 *  @param recvMsg   输出缓冲
 *  @param msgMaxCnt 最大条数
 *  @return 实际取到的条数 (0 = 无新报文)
 */
uint8_t can0_getMsg(Can_recv_msg_st *recvMsg, uint8_t msgMaxCnt)
{
	uint8_t i = 0;
	uint8_t j = 0;
	if (recvMsg == NULL || msgMaxCnt == 0)
	{
		return 0;
	}
	for (; ((i < s_can0_rcv_msgId_count) && (i < msgMaxCnt)); i++)
	{
		if (g_can0_recvMsg[i].msgRcvFlag > 0)
		{
			recvMsg[j].msgID = g_can0_recvMsg[i].msgID;
			recvMsg[j].msgRcvFlag = 1;
			memcpy(recvMsg[j].msg, g_can0_recvMsg[i].msg, 8);
			g_can0_recvMsg[i].msgRcvFlag = 0;
			j++;
		}
	}
	return j;
}

/** @brief CAN1 批量取接收报文 (取走即清接收标志)
 *  @param recvMsg   输出缓冲
 *  @param msgMaxCnt 最大条数
 *  @return 实际取到的条数 (0 = 无新报文)
 */
uint8_t can1_getMsg(Can_recv_msg_st *recvMsg, uint8_t msgMaxCnt)
{
	uint8_t i = 0;
	uint8_t j = 0;
	if (recvMsg == NULL || msgMaxCnt == 0)
	{
		return 0;
	}
	for (; ((i < s_can1_rcv_msgId_count) && (i < msgMaxCnt)); i++)
	{
		if (g_can1_recvMsg[i].msgRcvFlag > 0)
		{
			recvMsg[j].msgID = g_can1_recvMsg[i].msgID;
			recvMsg[j].msgRcvFlag = 1;
			memcpy(recvMsg[j].msg, g_can1_recvMsg[i].msg, 8);
			g_can1_recvMsg[i].msgRcvFlag = 0;
			j++;
		}
	}
	return j;
}


/** @brief CAN0 发送 (ID 须在发送白名单内, 否则直接返回 1)
 *  @param msgId  报文 ID (≤0x7FF 标准帧, 否则扩展帧)
 *  @param msg    数据 (不足 8 字节由 0xFF 填充)
 *  @param msgLen 有效长度 (1~8)
 *  @return 0 = 成功; 1 = 参数错误或不在白名单; 2 = 发送缓冲/HW 失败
 */
uint8_t can0_sendMsg(uint32_t msgId, uint8_t *msg, uint8_t msgLen)
{
	uint8_t bufIndx;
	
	cy_stc_canfd_msg_t stcMsg;
	uint8_t msgMask[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	
	if (msg == NULL || msgLen > 8 || msgLen == 0)
	{
		return 1;
	}

	for (bufIndx = 0; bufIndx < s_can0_snd_msgId_count; bufIndx++)
	{
		if (msgId == g_can0_sendID_lst[bufIndx])break;
	}
	if (bufIndx == s_can0_snd_msgId_count)
	{
		return 1;
	}

	memcpy(msgMask, msg, msgLen);

	stcMsg.canFDFormat = false;
	stcMsg.idConfig.extended = true;
	if (msgId <= 0x7FF)
	{
		stcMsg.idConfig.extended = false;
	}
	stcMsg.idConfig.identifier = msgId;
	stcMsg.dataConfig.dataLengthCode = 8;   //不够8byte,填充0xFF
	stcMsg.dataConfig.data[0]  = (uint32_t)((msgMask[3] << 24) +(msgMask[2] << 16) +(msgMask[1] << 8) + msgMask[0]);
	stcMsg.dataConfig.data[1]  = (uint32_t)((msgMask[7] << 24) +(msgMask[6] << 16) +(msgMask[5] << 8) + msgMask[4]);
	
	if (Cy_CANFD_UpdateAndTransmitMsgBuffer(CAN_CHSS_TYPE, bufIndx, &stcMsg))
	{
		return 2;
	}
	return 0;
}

/** @brief CAN1 发送 (不做白名单检查, 轮询 32 个 TX 缓冲找空闲)
 *  @param msgId  报文 ID (≤0x7FF 标准帧, 否则扩展帧)
 *  @param msg    数据 (不足 8 字节由 0xFF 填充)
 *  @param msgLen 有效长度 (1~8)
 *  @return 0 = 成功; 1 = 参数错误或 TX 缓冲全忙; 2 = 发送缓冲/HW 失败
 */
uint8_t can1_sendMsg(uint32_t msgId, uint8_t *msg, uint8_t msgLen)
{
	uint8_t i,j;
	cy_stc_canfd_msg_t stcMsg;
	static uint8_t prvBufIndx = 0;
	uint8_t msgMask[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	if (msg == NULL || msgLen > 8 || msgLen == 0)
	{
		return 1;
	}

	memcpy(msgMask, msg, msgLen);

	stcMsg.canFDFormat = false;
	stcMsg.idConfig.extended = false;
	if (msgId > 0x7FF)
		stcMsg.idConfig.extended = true;
	stcMsg.idConfig.identifier = msgId;
	stcMsg.dataConfig.dataLengthCode = 8;   //不够8byte,填充0xFF
	stcMsg.dataConfig.data[0]  = (uint32_t)((msgMask[3] << 24) +(msgMask[2] << 16) +(msgMask[1] << 8) + msgMask[0]);
	stcMsg.dataConfig.data[1]  = (uint32_t)((msgMask[7] << 24) +(msgMask[6] << 16) +(msgMask[5] << 8) + msgMask[4]);
	for (i = prvBufIndx; i < 32 + prvBufIndx; i++)
	{
		j = i;
		if (i >= 32){j = i - 32;}
		if (Cy_CANFD_GetTxBufferStatus(CAN_PRVT_TYPE, j) != CY_CANFD_TX_BUFFER_PENDING)
		{
			break;
		}
	}
	if (i == (32 + prvBufIndx))
	{
		return 1;
	}
	prvBufIndx = j;
	if (Cy_CANFD_UpdateAndTransmitMsgBuffer(CAN_PRVT_TYPE, prvBufIndx, &stcMsg))
	{
		return 2;
	}
	
	if (prvBufIndx < 31)
	{
		prvBufIndx++;
	}
	else
	{
		prvBufIndx = 0;
	}
	return 0;
}

/* ========================================================================== */
/*  8. 初始化与波特率                                                           */
/* ========================================================================== */

/** @brief CAN0/CAN1 初始化: 时钟分频 → 引脚 → 中断 → SDK 配置
 *  @note  白名单须在调用前由 can_info_cfg() 配好, 否则过滤器为空、收发全被拒。
 *         CAN0 波特率跟随 RTEComBaudRate (0 → 250k, 其他 → 500k)。
 */
void can_init(void)
{
	periph_divider(CAN_CHSS_PCLK, CY_SYSCLK_DIV_8_BIT, DIV_NO_CAN_CHSS, CAN_TARGET_FREQ);
	periph_divider(CAN_PRVT_PCLK, CY_SYSCLK_DIV_8_BIT, DIV_NO_CAN_PRVT, CAN_TARGET_FREQ);
	
	Cy_CANFD_DeInit(CAN_CHSS_TYPE);
	Cy_CANFD_DeInit(CAN_PRVT_TYPE);

	Cy_GPIO_Multi_Pin_Init(g_can_pin_cfg, CAN_PORT_NUM);

	Cy_SysInt_InitIRQ(&g_can0_irq_cfg);
	Cy_SysInt_SetSystemIrqVector(g_can0_irq_cfg.sysIntSrc, can0_interruptHandler);
	NVIC_SetPriority(CAN_CHSS_IRQ_INDX, 0);
	NVIC_ClearPendingIRQ(CAN_CHSS_IRQ_INDX);
	NVIC_EnableIRQ(CAN_CHSS_IRQ_INDX);

	Cy_SysInt_InitIRQ(&g_can1_irq_cfg);
	Cy_SysInt_SetSystemIrqVector(g_can1_irq_cfg.sysIntSrc, can1_interruptHandler);
	NVIC_SetPriority(CAN_PRVT_IRQ_INDX, 0);
	NVIC_ClearPendingIRQ(CAN_PRVT_IRQ_INDX);
	NVIC_EnableIRQ(CAN_PRVT_IRQ_INDX);
	
	canCfg.rxCallback = can_rxMsgCallback;
	canCfg.sidFilterConfig.numberOfSIDFilters = sizeof(g_can0_stdIdFilter) / sizeof(cy_stc_id_filter_t);
	canCfg.sidFilterConfig.sidFilter = g_can0_stdIdFilter;
	canCfg.extidFilterConfig.numberOfEXTIDFilters = sizeof(g_can0_extIdFilter) / sizeof(cy_stc_extid_filter_t);
	canCfg.extidFilterConfig.extidFilter = g_can0_extIdFilter;
	canCfg.extidFilterConfig.extIDANDMask = 0x1fffffff;
	//默认500KHz
	if (RTEComBaudRate == 0)  //设置为250KHz
	{
		canCfg.bitrate.prescaler	 = 20u - 1u;  // cclk/20, When using 250kbps, 1bit = 8tq
		canCfg.bitrate.timeSegment1	 = 5u - 1u; // tseg1 = 5tq
		canCfg.bitrate.timeSegment2	 = 2u - 1u;	// tseg2 = 2tq
		canCfg.bitrate.syncJumpWidth = 2u - 1u;	// sjw	 = 2tq
	}
	Cy_CANFD_Init(CAN_CHSS_TYPE, &canCfg);
	/* 补开 Bus_Off 状态中断: SDK 的 Cy_CANFD_Init 只使能 RX 中断 (DRXE/RF0NE/RF1NE), 且每次调用都会重写 unIE
	 * 不开此位则 busoff 时无中断 → s_busoff_flag0 恒为 0 → 恢复函数永不动作 */
	CAN_CHSS_TYPE->M_TTCAN.unIE.stcField.u1BOE = 1u;

	canCfg1.rxCallback = can1_rxMsgCallback;
	canCfg1.sidFilterConfig.numberOfSIDFilters = sizeof(g_can1_stdIdFilter) / sizeof(cy_stc_id_filter_t);
	canCfg1.sidFilterConfig.sidFilter = g_can1_stdIdFilter;
	canCfg1.extidFilterConfig.numberOfEXTIDFilters = sizeof(g_can1_extIdFilter) / sizeof(cy_stc_extid_filter_t);
	canCfg1.extidFilterConfig.extidFilter = g_can1_extIdFilter;
	canCfg1.extidFilterConfig.extIDANDMask = 0x1fffffff;
	Cy_CANFD_Init(CAN_PRVT_TYPE, &canCfg1);
	/* 补开 Bus_Off 状态中断: SDK 的 Cy_CANFD_Init 只使能 RX 中断 (DRXE/RF0NE/RF1NE), 且每次调用都会重写 unIE
	 * 不开此位则 busoff 时无中断 → s_busoff_flag0 恒为 0 → 恢复函数永不动作 */
	CAN_PRVT_TYPE->M_TTCAN.unIE.stcField.u1BOE = 1u;
}

/** @brief CAN0 波特率设置 (只改 canCfg, 需重新 can_init 才生效)
 *  @param bitRateFlag 0 = 1M, 1 = 500k, 2 = 250k, 3 = 125k, 其他 = 500k
 */
void can0_bitrate_set(uint8_t bitRateFlag)
{
	if (bitRateFlag == 0)
	{
		canCfg.bitrate.prescaler	 = 5u - 1u;	// cclk/5, When using 1Mbps, 1bit = 8tq
		canCfg.bitrate.timeSegment1	 = 5u - 1u; // tseg1 = 5tq
		canCfg.bitrate.timeSegment2	 = 2u - 1u;	// tseg2 = 2tq
		canCfg.bitrate.syncJumpWidth = 2u - 1u;	// sjw	 = 2tq
	}
	else if (bitRateFlag == 1)
	{
		canCfg.bitrate.prescaler	 = 10u - 1u;  // cclk/10, When using 500kbps, 1bit = 8tq
		canCfg.bitrate.timeSegment1	 = 5u - 1u; // tseg1 = 5tq
		canCfg.bitrate.timeSegment2	 = 2u - 1u;	// tseg2 = 2tq
		canCfg.bitrate.syncJumpWidth = 2u - 1u;	// sjw	 = 2tq
	}
	else if (bitRateFlag == 2)
	{
		canCfg.bitrate.prescaler	 = 20u - 1u;  // cclk/20, When using 250kbps, 1bit = 8tq
		canCfg.bitrate.timeSegment1	 = 5u - 1u; // tseg1 = 5tq
		canCfg.bitrate.timeSegment2	 = 2u - 1u;	// tseg2 = 2tq
		canCfg.bitrate.syncJumpWidth = 2u - 1u;	// sjw	 = 2tq
	}
	else if (bitRateFlag == 3)
	{
		canCfg.bitrate.prescaler	 = 40u - 1u;  // cclk/40, When using 125kbps, 1bit = 8tq
		canCfg.bitrate.timeSegment1	 = 5u - 1u; // tseg1 = 5tq
		canCfg.bitrate.timeSegment2	 = 2u - 1u;	// tseg2 = 2tq
		canCfg.bitrate.syncJumpWidth = 2u - 1u;	// sjw	 = 2tq
	}
	else
	{
		canCfg.bitrate.prescaler	 = 10u - 1u;  // cclk/10, When using 500kbps, 1bit = 8tq
		canCfg.bitrate.timeSegment1	 = 5u - 1u; // tseg1 = 5tq
		canCfg.bitrate.timeSegment2	 = 2u - 1u;	// tseg2 = 2tq
		canCfg.bitrate.syncJumpWidth = 2u - 1u;	// sjw	 = 2tq
	}
}

/* ========================================================================== */
/*  9. 对外查询                                                                 */
/* ========================================================================== */

/** @brief 读 busoff 中断状态捕获值 (中断里锁存, 0 = 正常)
 *  @param canX 0 = CAN0, 1 = CAN1
 *  @return unIR 原值; 调用方按需自行掩码判断
 *  @note   供 PSWdebug 等外部模块使用, 避免手写 extern 直接访问内部变量。
 */
uint32_t Can_GetBusoffFlag(uint8_t canX)
{
	if (canX == 1u)
	{
		return s_busoff_flag1;
	}
	return s_busoff_flag0;
}

/** @brief 读发送白名单条数 (由 can_info_cfg 配置, transDirect=1 时记录)
 *  @param canX 0 = CAN0, 1 = CAN1
 *  @note  CAN1 当前不做发送过滤 (见 can1_sendMsg), 条数仅作记录/调试查询。
 */
uint8_t Can_GetSendWhitelistCount(uint8_t canX)
{
	if (canX == 1u)
	{
		return s_can1_snd_msgId_count;
	}
	return s_can0_snd_msgId_count;
}