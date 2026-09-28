/**
 * @file    can.c
 * @brief   CAN 通讯驱动 — CAN0/CAN1 (Classic CAN, 500kbps) + 接收中断 + busoff 恢复
 *
 * @details 两个实例 (引脚/中断宏定义见 can.h):
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

uint32_t g_can0_sendID_lst[32] = {0};
uint8_t g_can0_snd_msgId_count;

uint32_t g_can1_sendID_lst[32] = {0};
uint8_t g_can1_snd_msgId_count;

Can_recv_msg_st g_can0_recvMsg[32];
uint8_t g_can0_rcv_msgId_count;

Can_recv_msg_st g_can1_recvMsg[32];
uint8_t g_can1_rcv_msgId_count;

//底盘CAN标准ID过滤配置
static cy_stc_id_filter_t g_can0_stdIdFilter[32];

//底盘CAN扩展ID过滤配置
static cy_stc_extid_filter_t g_can0_extIdFilter[32];


//私有CAN标准ID过滤配置
static cy_stc_id_filter_t g_can1_stdIdFilter[32];

//私有CAN扩展ID过滤配置
static cy_stc_extid_filter_t g_can1_extIdFilter[32];


//can全局配置
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

//can全局配置
cy_stc_canfd_config_t canCfg1 = 
{
	.txCallback     = NULL, // Unused.
	.rxCallback     = NULL,
	.rxFifoWithTopCallback = NULL,//CAN_RxFifoWithTopCallback,
	.statusCallback = NULL, // Un-supported now
	.errorCallback  = NULL, // Un-supported now
	.canFDMode      = false, // Use standard CAN mode

     // 40 MHz   500KHz
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


//引脚配置
static const cy_stc_gpio_pin_prt_config_t g_can_pin_cfg[] =
{
//       portReg,             pinNum,  outVal,   driveMode,         hsiom,      intEdge,intMask,vtrip,slewRate,driveSel,vregEn,ibufMode,vtripSel,vrefSel,vohSel
    { CAN_CHSS_RX_PORT, CAN_CHSS_RX_PIN, 0ul, CY_GPIO_DM_HIGHZ,  CAN_CHSS_RX_MUX, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul}, /* CAN0_2 RX -- CHASSIS RX*/
    { CAN_CHSS_TX_PORT, CAN_CHSS_TX_PIN, 1ul, CY_GPIO_DM_STRONG, CAN_CHSS_TX_MUX, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul}, /* CAN0_2 TX -- CHASSIS TX*/
    { CAN_PRVT_RX_PORT, CAN_PRVT_RX_PIN, 0ul, CY_GPIO_DM_HIGHZ,  CAN_PRVT_RX_MUX, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul}, /* CAN0_2 RX -- PRIVATE RX*/
    { CAN_PRVT_TX_PORT, CAN_PRVT_TX_PIN, 1ul, CY_GPIO_DM_STRONG, CAN_PRVT_TX_MUX, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul}, /* CAN0_2 TX -- PRIVATE TX*/

};
#define CAN_PORT_NUM (sizeof(g_can_pin_cfg)/sizeof(g_can_pin_cfg[0]))

//底盘CAN接收中断配置
cy_stc_sysint_irq_t g_can0_irq_cfg = {
	.sysIntSrc = CAN_CHSS_IRQN,
	.intIdx = CAN_CHSS_IRQ_INDX,
	.isEnabled = true,
};

 //私有CAN接收中断配置
 cy_stc_sysint_irq_t g_can1_irq_cfg = {
	.sysIntSrc = CAN_PRVT_IRQN,
	.intIdx = CAN_PRVT_IRQ_INDX,
	.isEnabled = true,
 };

/* 调试: CAN0 接收中断回调计数 (main_cm4.c CAN_TEST 测试用) */
volatile uint32_t g_can0_rx_cb_cnt = 0u;
volatile uint32_t g_can1_rx_cb_cnt = 0u;

//can接收回调函数
static void can_rxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_msg_t* pstcCanFDmsg)
{
	uint8_t i,j;
	uint8_t byte4Count = 0;

	g_can0_rx_cb_cnt++;

	if (pstcCanFDmsg->dataConfig.dataLengthCode != 8)
	{
		return;
	}

	for (i = 0; i < g_can0_rcv_msgId_count; i++)
	{
		if (pstcCanFDmsg->idConfig.identifier == g_can0_recvMsg[i].msgID)
		{
			break;
		}
	}

	//if (i == g_can_msgId_count || g_can_recvMsg[i].msgRcvFlag == true)  //旧数据未处理，新数据丢掉
	if (i == g_can0_rcv_msgId_count)
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

//can1接收回调函数
static void can1_rxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_msg_t* pstcCanFDmsg)
{
	uint8_t i,j;
	uint8_t byte4Count = 0;

	g_can1_rx_cb_cnt++;

	if (pstcCanFDmsg->dataConfig.dataLengthCode != 8)
	{
		return;
	}

	for (i = 0; i < g_can1_rcv_msgId_count; i++)
	{
		if (pstcCanFDmsg->idConfig.identifier == g_can1_recvMsg[i].msgID)
		{
			break;
		}
	}

	//if (i == g_can_msgId_count || g_can_recvMsg[i].msgRcvFlag == true)  //旧数据未处理，新数据丢掉
	if (i == g_can1_rcv_msgId_count)
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

uint32_t g_busoff_flag0 = 0;
uint8_t resume_count0 = 0;
uint8_t resume_time_ms0 = 0;
uint8_t g_max_resume_time0 = 5;


void can0_busoff_resume(void)
{
	if ((g_busoff_flag0 & 0x3ff7E0EE) > 0)
	//if ((g_busoff_flag0 & 0x2000000) > 0)
	{
		if (resume_count0 >= 10)
		{
			//return;
			g_max_resume_time0 = 100;
		}
		if (resume_time_ms0 < g_max_resume_time0)
		{
			resume_time_ms0++;
		}
		else
		{
			resume_time_ms0=0;
			if (resume_count0 < 10)
				resume_count0++;
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
			 * 不开此位则 busoff 时无中断 → g_busoff_flag0 恒为 0 → 恢复函数永不动作 */
			CAN_CHSS_TYPE->M_TTCAN.unIE.stcField.u1BOE = 1u;

			g_busoff_flag0 = 0;
		}
	}
	else
	{
		resume_count0 = 0;
		resume_time_ms0 = 0;
		g_max_resume_time0 = 5;
	}
}


//底盘CAN中断处理函数：获取buff数据，并调用接收回调函数处理报文
static void can0_interruptHandler(void)
{
	g_busoff_flag0 = CAN_CHSS_TYPE->M_TTCAN.unIR.u32Register;
	//uart_printf("int handler %x\r\n", g_busoff_flag0);
	Cy_CANFD_IrqHandler(CAN_CHSS_TYPE);
	CAN_CHSS_TYPE->M_TTCAN.unIR.u32Register = g_busoff_flag0;
}

uint32_t g_busoff_flag1 = 0;
uint8_t resume_count1 = 0;
uint8_t resume_time_ms1 = 0;
uint8_t g_max_resume_time1 = 5;


void can1_busoff_resume(void)
{
	//if ((g_busoff_flag1 & 0x2000000) > 0)
	if ((g_busoff_flag1 & 0x3ff7E0EE) > 0)
	{
		if (resume_count1 >= 10)
		{
			//return;
			g_max_resume_time1 = 100;
		}
		if (resume_time_ms1 < g_max_resume_time1)
		{
			resume_time_ms1++;
		}
		else
		{
			resume_time_ms1=0;
			if (resume_count1 < 10)
				resume_count1++;
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
			 * 不开此位则 busoff 时无中断 → g_busoff_flag0 恒为 0 → 恢复函数永不动作 */
			CAN_PRVT_TYPE->M_TTCAN.unIE.stcField.u1BOE = 1u;

			g_busoff_flag1 = 0;
		}
	}
	else
	{
		resume_count1 = 0;
		resume_time_ms1 = 0;
		g_max_resume_time1 = 5;
	}
}



//私有CAN中断处理函数：获取buff数据，并调用接收回调函数处理报文
static void can1_interruptHandler(void)
{
	g_busoff_flag1 = CAN_PRVT_TYPE->M_TTCAN.unIR.u32Register;
	//uart_printf("int handler %x\r\n", g_busoff_flag1);
	Cy_CANFD_IrqHandler(CAN_PRVT_TYPE);
	CAN_PRVT_TYPE->M_TTCAN.unIR.u32Register = g_busoff_flag1;
}

//canX: 0 - CAN_CHASSIS; 1 - CAN_PRIVATE
//transDirect: 0 - recive; 1 - send
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
			g_can0_rcv_msgId_count = 0;
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
				g_can0_recvMsg[g_can0_rcv_msgId_count].msgID = canIDList[i];
				g_can0_recvMsg[g_can0_rcv_msgId_count].msgRcvFlag = 0;
				g_can0_rcv_msgId_count++;
			}
		}
		else   //发送BUF
		{
			memset(g_can0_sendID_lst, 0x00, sizeof(g_can0_sendID_lst));
			if (idCount > 32)
			{
				memcpy(g_can0_sendID_lst, canIDList, 32*sizeof(g_can0_sendID_lst[0]));
				g_can0_snd_msgId_count = 32;
			}
			else
			{
				memcpy(g_can0_sendID_lst, canIDList, idCount*sizeof(g_can0_sendID_lst[0]));
				g_can0_snd_msgId_count = idCount;
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
			g_can1_rcv_msgId_count = 0;
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
				g_can1_recvMsg[g_can1_rcv_msgId_count].msgID = canIDList[i];
				g_can1_recvMsg[g_can1_rcv_msgId_count].msgRcvFlag = 0;
				g_can1_rcv_msgId_count++;
			}
		}
		else    //发送BUF
		{
			memset(g_can1_sendID_lst, 0x00, sizeof(g_can1_sendID_lst));
			if (idCount > 128)
			{
				memcpy(g_can1_sendID_lst, canIDList, 128*sizeof(g_can1_sendID_lst[0]));
				g_can1_snd_msgId_count = 128;
			}
			else
			{
				memcpy(g_can1_sendID_lst, canIDList, idCount*sizeof(g_can1_sendID_lst[0]));
				g_can1_snd_msgId_count = idCount;
			}
		}
	}
}

uint8_t can0_getMsg(Can_recv_msg_st *recvMsg, uint8_t msgMaxCnt)
{
	uint8_t i = 0;
	uint8_t j = 0;
	if (recvMsg == NULL || msgMaxCnt == 0)
	{
		return 0;
	}
	for (; ((i < g_can0_rcv_msgId_count) && (i < msgMaxCnt)); i++)
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

uint8_t can1_getMsg(Can_recv_msg_st *recvMsg, uint8_t msgMaxCnt)
{
	uint8_t i = 0;
	uint8_t j = 0;
	if (recvMsg == NULL || msgMaxCnt == 0)
	{
		return 0;
	}
	for (; ((i < g_can1_rcv_msgId_count) && (i < msgMaxCnt)); i++)
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


//底盘CAN发送函数
uint8_t can0_sendMsg(uint32_t msgId, uint8_t *msg, uint8_t msgLen)
{
	uint8_t bufIndx;
	
	cy_stc_canfd_msg_t stcMsg;
	uint8_t msgMask[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	
	if (msg == NULL || msgLen > 8 || msgLen == 0)
	{
		return 1;
	}

	for (bufIndx = 0; bufIndx < g_can0_snd_msgId_count; bufIndx++)
	{
		if (msgId == g_can0_sendID_lst[bufIndx])break;
	}
	if (bufIndx == g_can0_snd_msgId_count)
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

//私有CAN发送函数
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
	
#if 0  //发送去除过滤
	for (i = 0; i < g_can1_snd_msgId_count; i++)
	{
		if (g_can1_sendID_lst[i] == msgId)break;
	}
	if (i == g_can1_snd_msgId_count)
	{
		return 1;
	}
#endif
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

//初始化函数
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
	 * 不开此位则 busoff 时无中断 → g_busoff_flag0 恒为 0 → 恢复函数永不动作 */
	CAN_CHSS_TYPE->M_TTCAN.unIE.stcField.u1BOE = 1u;

	canCfg1.rxCallback = can1_rxMsgCallback;
	canCfg1.sidFilterConfig.numberOfSIDFilters = sizeof(g_can1_stdIdFilter) / sizeof(cy_stc_id_filter_t);
	canCfg1.sidFilterConfig.sidFilter = g_can1_stdIdFilter;
	canCfg1.extidFilterConfig.numberOfEXTIDFilters = sizeof(g_can1_extIdFilter) / sizeof(cy_stc_extid_filter_t);
	canCfg1.extidFilterConfig.extidFilter = g_can1_extIdFilter;
	canCfg1.extidFilterConfig.extIDANDMask = 0x1fffffff;
	Cy_CANFD_Init(CAN_PRVT_TYPE, &canCfg1);
	/* 补开 Bus_Off 状态中断: SDK 的 Cy_CANFD_Init 只使能 RX 中断 (DRXE/RF0NE/RF1NE), 且每次调用都会重写 unIE
	 * 不开此位则 busoff 时无中断 → g_busoff_flag0 恒为 0 → 恢复函数永不动作 */
	CAN_PRVT_TYPE->M_TTCAN.unIE.stcField.u1BOE = 1u;
}

/*
 * 功能：公共can比特率设置
 * 参数：bitRateFlag：0 - 1MHz; 1 - 500KHz; 2 - 250KHz; 3 - 125KHz; 其他 - 500KHz;
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
