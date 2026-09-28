/**
 * @file    PSWdebug.c
 * @brief   测试用例框架 — 函数指针表 + CAN 动态激活 + 10ms 周期执行
 *
 * @details 激活机制 (PSWdebug 每 10ms 调用):
 *          ① 读取 RTEfDbgMsgSW3 (CAN 下发的测试编号)
 *          ② 匹配 g_debugCase_list[] → 注册函数指针到 g_debugFuncList[]
 *          ③ 遍历 g_debugFuncList[], 逐个执行已注册的测试函数
 *
 * @details 测试编号约定 (_XX_XXX_testCase), 与上位机用例文档一致:
 *          +-------+-----------+--------------------------------------------------+
 *          | 编号   | 大类       | 说明                                              |
 *          +-------+-----------+--------------------------------------------------+
 *          | 0x     | 通信       | 00=CAN0 01=UART 02=CAN白名单 03=CANbusoff        |
 *          | 1x     | 系统       | 10=软件复位 11=读复位原因                         |
 *          | 2x     | 看门狗     | 20=硬狗超时 21=软狗超时                           |
 *          | 3x     | 定时器     | 30=10ms 主任务周期                                |
 *          | 4x     | 电源/GPIO  | 40=继电器+UB 41=开关输入 42=继电器故障             |
 *          | 6x     | EEPROM     | 60=读写耗时 (前 256B, 测完恢复)                    |
 *          | 7x     | 传感器     | 70=VPOWER 71=压力 72=轮速传感器故障                |
 *          | 8x     | 轮速值     | 80=FL/FR/RL/RR 81=XL/XR                          |
 *          | 9x/Ax  | 阀/芯片诊断 | 90=低边开关 91=全阀PWM扫描 92~94=阀故障 A0/A1=芯片 |
 *          +-------+-----------+--------------------------------------------------+
 *
 * @note    计时观测点 P22.0（复用左后排气阀 LRO 的 PWM 脚）: 需要示波器测时序的用例 (30/60/20/21 等) 通过
 *          TestMark_Ctrl() 输出高低电平; 该脚首次被调用时脱离 TCPWM 切为 GPIO 推挽输出,
 *          详见 gpio.c 的 TestMark_Ctrl() 说明。
 *          注: 旧注释误写 P12.1, 该脚现已改作独立 SAR1(AN5) 模拟输入 (见 hw_rev.h)。
 */

#include "cmn.h"
#include "can.h"
#include "wheel_speed.h"
#include "adc.h"
#include "bts724g.h"
#include "spi_eeprom.h"
#include "eeprom.h"
#include "rte_psw.h"
#include "psw_data.h"
#include "gpio.h"
#include "RTE.h"
#include <string.h>
#include "PSWdebug.h"

/* TR_ASR 阀控制 — RTE.h 未导出该接口 (rte_psw.c 保留了实现), 调试用例需要, 此处显式声明 */
extern void ASRValActX(uint16_t period, uint16_t htime);

/* ======================================================================== */
/*  调试报文出口                                                               */
/*                                                                            */
/*  CANDbgMsgSend 属通讯层(COM)接口 — 总工程由 COM.lib 提供, 走 COM 的调试通道。  */
/*  纯 PSW 的整机工程没有通讯层, 故此处给一个 __weak 兜底实现: 走调试 CAN(can1,   */
/*  无发送白名单), 保证测试用例在整机上也能上报; 上层提供强定义时链接器自动改用上层。 */
/* ======================================================================== */
__weak void CANDbgMsgSend(uint32_t id, uint8_t *CANTxPacket)
{
    (void)can1_sendMsg(id, CANTxPacket, 8u);
}

/* ======================================================================== */
/*  框架常量和类型                                                             */
/* ======================================================================== */

#define BSW_SEND_ID_BASE    0x1CF00300u
#define MAX_DEBUG_CNT       15u

typedef void (*DebugFunc)(void);

typedef struct
{
    uint8_t  debugIndx;           /* 测试编号 (CAN 激活用的开关值) */
    DebugFunc debugFunc;          /* 测试函数指针                   */
    uint8_t  sendIdCnt;           /* 结果上报 CAN ID 数量           */
    uint32_t sendIdList[15];      /* 结果上报 CAN ID 列表           */
} debug_list_t;

/* ======================================================================== */
/*  激活运行时状态                                                             */
/* ======================================================================== */

static uint8_t   g_debugOpenCnt;                  /* 已激活的测试用例数     */
static DebugFunc g_debugFuncList[MAX_DEBUG_CNT];  /* 已激活函数指针数组     */
static uint8_t   g_debugOpenIndx[MAX_DEBUG_CNT];  /* 已激活的测试编号(防重) */

/* ======================================================================== */
/*  CAN 上报包装                                                               */
/* ======================================================================== */

/**
 * @brief 通过 CAN0 发送测试结果帧
 * @param id       相对于 BSW_SEND_ID_BASE 的偏移 ID
 * @param data     数据指针
 * @param dataLen  有效数据长度 (≤8)
 */
static void canPSWDbgMsgSend(uint32_t id, const uint8_t *data, uint8_t dataLen)
{
    uint8_t sendData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for (uint8_t i = 0u; i < dataLen; i++)
    {
        sendData[i] = data[i];
    }
    CANDbgMsgSend(BSW_SEND_ID_BASE + id, sendData);
}

/* ======================================================================== */
/*  测试用例函数                                                               */
/* ======================================================================== */

/* ---- 0x 通信 ---- */

/** @brief 00 — CAN0 通信 (占位) */
void _00_CAN0_testCase(void)
{
    /* TODO: CAN0 */
}

/** @brief 01 — UART 通信 (占位) */
void _01_UART_testCase(void)
{
    /* TODO: UART */
}

/* ===== 02 — CAN 硬件白名单(底层过滤)有效性 =====
 * 验证芯片硬件 ID filter 是否生效。判据: 名单内 ID 能被收到 = 白名单有效。
 *   (芯片配置 nonMatchingFramesStandard = REJECT, 白名单未配好时名单内 ID 也会被拒收)
 * 注意: 不调用 can_info_cfg, 避免覆盖通讯层在 can_init 前配好的硬件白名单。
 * 外部 CAN 分析仪往 CAN0 发名单内 ID:
 *   - 发 0x18FEF100 → extCnt 增长 = 扩展 ID 硬件过滤生效
 *   - 发 0x7A1      → stdCnt 增长 = 标准 ID 硬件过滤生效
 * CAN 0x02: [extCnt_L, extCnt_H, stdCnt_L, stdCnt_H] */
void _02_CAN_Filter_testCase(void)
{
    static uint16_t cycle  = 0u;
    static uint16_t extCnt = 0u;   /* 0x18FEF100 扩展ID 接收计数 */
    static uint16_t stdCnt = 0u;   /* 0x7A1 标准ID 接收计数 */
    static Can_recv_msg_st rcv[32];
    uint8_t n, i;
    uint8_t msg[4] = {0};

    n = can0_getMsg(rcv, 32u);
    for (i = 0u; i < n; i++)
    {
        if (rcv[i].msgID == 0x18FEF100u)  { extCnt++; }
        else if (rcv[i].msgID == 0x7A1u)  { stdCnt++; }
    }

    cycle++;
    if ((cycle % 100u) == 0u)
    {
        msg[0] = (uint8_t)extCnt;
        msg[1] = (uint8_t)(extCnt >> 8);
        msg[2] = (uint8_t)stdCnt;
        msg[3] = (uint8_t)(stdCnt >> 8);
        canPSWDbgMsgSend(0x02u, msg, 4u);
    }
}

/* ===== 03 — CAN busoff 恢复时间测试 (can0 底盘CAN + can1 私有CAN) =====
 * P22.0(直测): 任一通道 busoff 期间输出高电平, 全部恢复后拉低,
 *   示波器看 P22.0 高电平脉宽 = 当前被测通道的 busoff 恢复时间(同一时刻只短接一个通道)
 * 触发: 硬件制造总线故障(短接 CAN_H/CAN_L 或断开总线), TEC 累积到 256 进入 busoff
 * 恢复机制: can0_busoff_resume()/can1_busoff_resume() 每 10ms 调用,
 *   检测到 busoff 后等 g_max_resume_time0/1 个周期再重新 init
 * CAN 0x03: [busoffState0, busoffState1]  0=正常 1=busoff中
 *   发送通道自适应: can0 正常时走 can0; 短接 can0 测 busoff 时改走 can1,
 *   保证报文在 busoff 期间仍能实时上报 */
void _03_CAN_busoff_testCase(void)
{
    extern uint32_t g_busoff_flag0;
    extern uint32_t g_busoff_flag1;
    static uint8_t busoffState0 = 0u;
    static uint8_t busoffState1 = 0u;
    uint8_t msg[2] = {0};

    /* 掩码须与 can.c 的 can0/1_busoff_resume 一致 */
    if ((g_busoff_flag0 & 0x3FF7E0EEu) != 0u) { busoffState0 = 1u; }
    else                                     { busoffState0 = 0u; }

    if ((g_busoff_flag1 & 0x3FF7E0EEu) != 0u) { busoffState1 = 1u; }
    else                                     { busoffState1 = 0u; }

    if ((busoffState0 != 0u) || (busoffState1 != 0u)) { TestMark_Ctrl(1u); }
    else                                             { TestMark_Ctrl(0u); }

    msg[0] = busoffState0;
    msg[1] = busoffState1;

    if ((g_busoff_flag0 & 0x3FF7E0EEu) != 0u)
    {
        can1_sendMsg(0x1CF00303u, msg, 2u);   /* can0 busoff 时改走 can1 上报 */
    }
    else
    {
        canPSWDbgMsgSend(0x03u, msg, 2u);
    }
}

/* ---- 1x 系统 ---- */

/** @brief 10 — 软件复位 (复位前每秒上报计数, 100 拍后触发 system_reboot) */
void _10_softReset_testCase(void)
{
    static uint8_t rstFlag = 0;
    rstFlag++;
    if (rstFlag < 100u)
    {
        TestMark_Ctrl(1u);
        canPSWDbgMsgSend(0x10u, (uint8_t *)&rstFlag, 1u);
    }
    else
    {
        system_reboot();
    }
}

/** @brief 11 — 读复位原因 (1 = 看门狗复位) */
void _11_softReset_testCase(void)
{
    canPSWDbgMsgSend(0x11u, (uint8_t *)&resetReason, 4u);
}

/* ---- 2x 看门狗 ---- */

/* ===== 20 — 硬看门狗超时时间测试 =====
 * 硬狗 = 外部 WDT, MCU 翻转 P5.0 喂狗; 超时时间由外部芯片(型号/外围 RC)决定, 软件不可配, 只能实测
 * 方法: P22.0 拉高后进入死循环, 循环内只喂软狗保活、不喂硬狗;
 *   死循环同时阻塞主循环, 使 PSW_10ms_deal() 里的 wd_feed() 无法再喂硬狗
 * 判据: 示波器测 P22.0 高电平脉宽 = 硬看门狗超时时间 (复位后引脚回默认 → P22.0 变低)
 * ⚠ 关键点: 主循环每 10ms 调用 wd_feed() 会同时喂硬狗+软狗, 若不在此阻塞, 硬狗永远不会超时 */
void _20_WD_testCase(void)
{
    static uint16_t cnt = 0u;
    cnt++;
    canPSWDbgMsgSend(0x20u, (uint8_t *)&cnt, 2u);   /* 复位前上报计数 */

    TestMark_Ctrl(1u);

    for (;;)
    {
        Cy_WDT_ClearWatchdog();   /* 只喂软狗保活, 不喂硬狗 */
    }
}

/* ===== 21 — 软看门狗超时时间测试 =====
 * 软狗 = 芯片内部 WDT, SoftWD_init() 设 Upper Limit (cmn.c 标注约 50ms), 喂狗 Cy_WDT_ClearWatchdog()
 * 方法: P22.0 拉高后进入死循环, 循环内只翻转 P5.0 喂硬狗保活、不喂软狗
 * 判据: 示波器测 P22.0 高电平脉宽 = 软看门狗超时时间; 复位后可用例 0x11 读复位原因确认(=1) */
void _21_WD_testCase(void)
{
    static uint16_t cnt = 0u;
    cnt++;
    canPSWDbgMsgSend(0x21u, (uint8_t *)&cnt, 2u);

    TestMark_Ctrl(1u);

    for (;;)
    {
        Cy_GPIO_Inv(WD_HW_GPIO_PORT, WD_HW_GPIO_PIN);   /* 只喂硬狗保活, 不喂软狗 */
    }
}

/* ---- 3x 定时器 ---- */

/* ===== 30 — 10ms 主任务周期验证 =====
 * 每次调用直接翻转 P22.0(直测), 高低电平各占 1 个主任务周期
 * 主任务为 10ms 时: 高 10ms + 低 10ms, 方波周期 20ms
 * 判据: 方波周期 ÷ 2 = PSW_10ms_deal() 调度周期; 0x1CF00330 报文帧间隔也 = 主任务周期
 * 若出现高低不对称(如 20ms/1ms), 说明计时引脚还受其它外设影响, 应检查是否为 GPIO 直控 */
void _30_10MStimer_testCase(void)
{
    static uint32_t timerCnt = 0u;
    timerCnt++;
    timerCnt = (timerCnt > 1u) ? 0u : 1u;
    TestMark_Ctrl((uint8_t)timerCnt);
    canPSWDbgMsgSend(0x30u, (uint8_t *)&timerCnt, 4u);
}

/* ---- 4x 电源/GPIO ---- */

/* ===== 40 — GPO 测试: RelayCtrl(继电器 P14.3) / UBCtrl(UB 供电 P13.6) =====
 * 三段循环(每段 100 拍 = 1s): 阶段0 全开 → 阶段1 全关 → 阶段2 恢复默认(全关)
 * ⚠ 关继电器会切断 VPOWER/UB 供电, 测试结束后需重新上电恢复 */
void _40_GPO_testCase(void)
{
    static uint16_t gpoFlag = 0u;
    uint8_t msg[4] = {0};

    gpoFlag++;
    if (gpoFlag >= 300u) { gpoFlag = 0u; }

    if (gpoFlag < 100u)          /* 阶段0: 全部打开 */
    {
        RelayCtrl(1u);
        UBCtrl(1u);
        msg[0] = 0u;
    }
    else if (gpoFlag < 200u)     /* 阶段1: 全部关闭 */
    {
        RelayCtrl(0u);
        UBCtrl(0u);
        msg[0] = 1u;
    }
    else                         /* 阶段2: 保持关闭 (恢复默认) */
    {
        RelayCtrl(0u);
        UBCtrl(0u);
        msg[0] = 2u;
    }

    msg[1] = (uint8_t)gpoFlag;
    msg[2] = (uint8_t)(gpoFlag >> 8);
    canPSWDbgMsgSend(0x40u, msg, 3u);
}

/** @brief 41 — 接插件开关状态读取 (X3-4 挂车ABS / X1-6 ESC / X1-5 坡起) */
void _41_GPI_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)Xn_pin_StaGet(3u, 4u);   /* X3-4: 挂车ABS开关 */
    data[1] = (uint8_t)Xn_pin_StaGet(1u, 6u);   /* X1-6: ESC开关 */
    data[2] = (uint8_t)Xn_pin_StaGet(1u, 5u);   /* X1-5: 坡起开关 */
    canPSWDbgMsgSend(0x41u, data, 3u);
}

/** @brief 42 — 继电器故障状态 (0-正常 1-故障) */
void _42_GPI_testCase(void)
{
    uint8_t gpiMsg[8] = {0};
    gpiMsg[0] = PSWErrRelay;
    canPSWDbgMsgSend(0x42u, gpiMsg, 1u);
}

/* ---- 6x EEPROM ---- */

/* ===== 60 — EEPROM 读写测试 (只动前 256 字节, 测完自动恢复) =====
 * 4 个阶段各在 P22.0 上输出一个高电平脉冲, 示波器测脉宽 = 该操作耗时:
 *   脉冲1 = 写 256 字节, 脉冲2 = 读 256 字节, 脉冲3 = 单字节写, 脉冲4 = 单字节读
 * 正确性校验: 写块用递增模式 buf[i]=i(能暴露地址错位/数据线故障), 写后读回逐字节比较;
 *   Eeprom_WriteBlock/WriteByte 内部已做读回校验, 本用例再读回兜底 + 上报不一致字节数
 * 前 256 字节 = ERR_CODE(128B) + ASW 配置(128B); 测试前先备份、测试后恢复, 避免破坏数据
 * 周期 300 拍(每拍 10ms, 4 个脉冲间隔约 0.5s, 便于示波器分段捕捉)
 * CAN 0x60: [phase, badCnt, badAddr_lo, badAddr_hi]
 *   phase 1=写256 2=读256校验 3=单字节写 4=单字节读校验 0=空闲; badCnt 0=全对 255=底层返回失败;
 *   badAddr = 第一个不一致地址(0xFFFF=无) */
void _60_EEPROM_testCase(void)
{
    static uint16_t eeFlag = 0u;
    static uint8_t  buf[256];
    static uint8_t  backup[256];
    uint8_t  msg[4]   = {0};
    uint8_t  phase    = 0u;
    uint8_t  badCnt   = 0u;
    uint16_t badAddr  = 0xFFFFu;
    uint16_t i;

    eeFlag++;
    if (eeFlag >= 300u) { eeFlag = 0u; }

    switch (eeFlag)
    {
    case 10u:                                  /* 先备份前 256 字节 */
        Eeprom_ReadBlock(0u, backup, 256u);
        break;

    case 50u:                                  /* ① 写 256 字节 (递增模式) */
        phase = 1u;
        for (i = 0u; i < 256u; i++) { buf[i] = (uint8_t)i; }
        TestMark_Ctrl(1u);                         /* 脉冲开始 */
        badCnt = (Eeprom_WriteBlock(0u, buf, 256u) == 256u) ? 0u : 255u; /* 返回 256 = 写后读回 3 次全对 */
        TestMark_Ctrl(0u);                         /* 脉冲结束, 脉宽 = 写 256 耗时 */
        break;

    case 100u:                                 /* ② 读 256 字节 + 逐字节校验 */
        phase = 2u;
        TestMark_Ctrl(1u);
        Eeprom_ReadBlock(0u, buf, 256u);
        TestMark_Ctrl(0u);                         /* 脉宽 = 读 256 耗时 */
        badCnt  = 0u;
        badAddr = 0xFFFFu;
        for (i = 0u; i < 256u; i++)
        {
            if (buf[i] != (uint8_t)i)
            {
                if (badCnt == 0u) { badAddr = i; }
                badCnt++;
            }
        }
        break;

    case 150u:                                 /* ③ 单字节写 0xA5 */
        phase = 3u;
        TestMark_Ctrl(1u);
        badCnt = Eeprom_WriteByte(0u, 0xA5u) ? 0u : 255u;  /* WriteByte 内部已读回校验 */
        TestMark_Ctrl(0u);                         /* 脉宽 = 单字节写耗时 */
        break;

    case 200u:                                 /* ④ 单字节读 + 校验 */
        phase = 4u;
        TestMark_Ctrl(1u);
        buf[0] = Eeprom_ReadByte(0u);
        TestMark_Ctrl(0u);                         /* 脉宽 = 单字节读耗时 */
        badCnt = (buf[0] == 0xA5u) ? 0u : 255u;
        break;

    case 250u:                                 /* 恢复前 256 字节原始数据 */
        Eeprom_WriteBlock(0u, backup, 256u);
        break;

    default:
        break;
    }

    msg[0] = phase;
    msg[1] = badCnt;
    msg[2] = (uint8_t)(badAddr & 0xFFu);
    msg[3] = (uint8_t)(badAddr >> 8);
    canPSWDbgMsgSend(0x60u, msg, 4u);
}

/* ---- 7x 传感器采集 (ADC) ---- */

/** @brief 70 — VPOWER 电压 + 报警 (小端字节序; 本板仅一路 VPOWER, vBat/vIgn 同源) */
void _70_ADC_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(PSWvIgn & 0xFFu);        /* vBat 低字节 (同 VPOWER) */
    data[1] = (uint8_t)(PSWvIgn >> 8);           /* vBat 高字节 */
    data[2] = 0u;                                 /* vBat警 无独立电池 */
    data[3] = 0u;
    data[4] = (uint8_t)(PSWvIgn & 0xFFu);        /* vIgn 低字节 (VPOWER) */
    data[5] = (uint8_t)(PSWvIgn >> 8);           /* vIgn 高字节 */
    data[6] = PSWErrVpwrLo;                      /* VPOWER 低压故障 */
    data[7] = PSWErrVpwrHi;                      /* VPOWER 高压故障 */
    canPSWDbgMsgSend(0x70u, data, 8u);
}

/** @brief 71 — 前后桥压力传感器 (小端字节序, 旧 DBC 格式) */
void _71_ADC_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(PSWBrkPreX2_7 & 0xFFu);        /* 前桥压力 低字节 */
    data[1] = (uint8_t)(PSWBrkPreX2_7 >> 8);           /* 前桥压力 高字节 */
    data[2] = PSWfErrPreX2_7;                          /* 前桥故障 */
    data[3] = 0u;
    data[4] = (uint8_t)(PSWBrkPreX3_10 & 0xFFu);       /* 后桥压力 低字节 */
    data[5] = (uint8_t)(PSWBrkPreX3_10 >> 8);          /* 后桥压力 高字节 */
    data[6] = PSWfErrPreX3_10;                         /* 后桥故障 */
    data[7] = 0u;
    canPSWDbgMsgSend(0x71u, data, 8u);
}

/** @brief 72 — 六通道轮速传感器故障 (旧 DBC 顺序: FL/FR/RL/RR/XL/XR) */
void _72_ADC_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = PSWErrWssOpenFL | (PSWErrWssShortFL << 1) | (PSWErrWssGapFL << 2);
    data[1] = PSWErrWssOpenFR | (PSWErrWssShortFR << 1) | (PSWErrWssGapFR << 2);
    data[2] = PSWErrWssOpenRL | (PSWErrWssShortRL << 1) | (PSWErrWssGapRL << 2);
    data[3] = PSWErrWssOpenRR | (PSWErrWssShortRR << 1) | (PSWErrWssGapRR << 2);
    data[4] = PSWErrWssOpenXL | (PSWErrWssShortXL << 1) | (PSWErrWssGapXL << 2);
    data[5] = PSWErrWssOpenXR | (PSWErrWssShortXR << 1) | (PSWErrWssGapXR << 2);
    canPSWDbgMsgSend(0x72u, data, 6u);
}

/* ---- 8x 轮速计算值 ---- */

/** @brief 80 — 轮速值 FL+FR+RL+RR (小端字节序, 旧 DBC 格式) */
void _80_PWM_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(PSWWheelSpeedFL & 0xFFu); data[1] = (uint8_t)(PSWWheelSpeedFL >> 8);
    data[2] = (uint8_t)(PSWWheelSpeedFR & 0xFFu); data[3] = (uint8_t)(PSWWheelSpeedFR >> 8);
    data[4] = (uint8_t)(PSWWheelSpeedRL & 0xFFu); data[5] = (uint8_t)(PSWWheelSpeedRL >> 8);
    data[6] = (uint8_t)(PSWWheelSpeedRR & 0xFFu); data[7] = (uint8_t)(PSWWheelSpeedRR >> 8);
    canPSWDbgMsgSend(0x80u, data, 8u);
}

/** @brief 81 — 轮速值 XL+XR (小端字节序, 旧 DBC 格式) */
void _81_PWM_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(PSWWheelSpeedXL & 0xFFu); data[1] = (uint8_t)(PSWWheelSpeedXL >> 8);
    data[2] = (uint8_t)(PSWWheelSpeedXR & 0xFFu); data[3] = (uint8_t)(PSWWheelSpeedXR >> 8);
    canPSWDbgMsgSend(0x81u, data, 4u);
}

/* ---- 9x/Ax 阀与芯片诊断 ---- */

/* ===== 90 — 低边开关测试 (前桥 P2.0 / 后桥 P6.2) =====
 * 前 100 拍打开, 后 100 拍关闭, 每拍回读状态上报
 * CAN 0x90: [P2.0状态, P6.2状态]  0=关 1=开 */
void _90_PWM_CMP_testCase(void)
{
    static uint16_t cnt = 0u;
    uint8_t msg[2] = {0};

    cnt++;
    if (cnt >= 200u) { cnt = 0u; }

    if (cnt < 100u)
    {
        InLowSideSwX4_16(1u);   /* P2.0 ON: 前桥ASR + 挂车ABS */
        OutLowSideSwX2_16(1u);  /* P6.2 ON: 后桥ASR */
    }
    else
    {
        InLowSideSwX4_16(0u);
        OutLowSideSwX2_16(0u);
    }

    msg[0] = InLowSideStX4_16();    /* P2.0 当前状态 */
    msg[1] = OutLowSideStX2_16();   /* P6.2 当前状态 */
    canPSWDbgMsgSend(0x90u, msg, 2u);
}

/* ===== 91 — 全阀 PWM 扫描 =====
 * 打开两路低边开关并屏蔽阀诊断, 4 相各 300 拍循环: 全关(0%) → 33% → 75% → 全开(100%)
 * 进气阀后段高(LOW→HIGH) / 排气阀前段高(HIGH→LOW), 周期/高电平时间单位 0.01ms */
void _91_PWM_CMP_testCase(void)
{
    static uint16_t cnt = 0;
    RTEfValWssTestForbit = 1;   /* 测试过程中不再检阀 */
    cnt++;
    if (cnt >= 1200u) { cnt = 0u; }

    InLowSideSwX4_16(1u);
    OutLowSideSwX2_16(1u);

    /* ---- Phase 0: 0% DC (全程关阀) ---- */
    if (cnt < 300u)
    {
        /* ABS 进气阀 (后段高 LOW→HIGH) */
        InValActFL(200u, 0u);   /* 前左进气 */
        InValActFR(200u, 0u);   /* 前右进气 */
        InValActRL(200u, 0u);   /* 后左进气 */
        InValActRR(200u, 0u);   /* 后右进气 */
        InValActXL(200u, 0u);   /* 辅左进气 */
        InValActXR(200u, 0u);   /* 辅右进气 */
        InValActX3_7(200u, 0u);   /* 21口 进气 */
        /* ABS 排气阀 (前段高 HIGH→LOW) */
        OutValActFL(200u, 0u);  /* 前左排气 */
        OutValActFR(200u, 0u);  /* 前右排气 */
        OutValActRL(200u, 0u);  /* 后左排气 */
        OutValActRR(200u, 0u);  /* 后右排气 */
        OutValActXL(200u, 0u);  /* 辅左排气 */
        OutValActXR(200u, 0u);  /* 辅右排气 */
        OutValActX3_10(2000u, 0u); /* 21口 排气 */
        /* 22口 阀 (与普通 ABS 阀极性一致) + TR_ASR */
        InValActX4_16(200u, 0u);   /* 22口 进气 */
        OutValActX2_16(200u, 0u);  /* 22口 排气 */
        ASRValActX(200u, 0u);      /* TR_ASR */
    }
    /* ---- Phase 1: 33% DC ---- */
    else if (cnt < 600u)
    {
        InValActFL(300u, 100u);
        InValActFR(300u, 100u);
        InValActRL(300u, 100u);
        InValActRR(300u, 100u);
        InValActXL(300u, 100u);
        InValActXR(300u, 100u);
        InValActX3_7(300u, 100u);
        OutValActFL(300u, 100u);
        OutValActFR(300u, 100u);
        OutValActRL(300u, 100u);
        OutValActRR(300u, 100u);
        OutValActXL(300u, 100u);
        OutValActXR(300u, 100u);
        OutValActX3_10(3000u, 1000u);
        InValActX4_16(300u, 100u);
        OutValActX2_16(300u, 100u);
        ASRValActX(300u, 100u);
    }
    /* ---- Phase 2: 75% DC ---- */
    else if (cnt < 900u)
    {
        InValActFL(400u, 300u);
        InValActFR(400u, 300u);
        InValActRL(400u, 300u);
        InValActRR(400u, 300u);
        InValActXL(400u, 300u);
        InValActXR(400u, 300u);
        InValActX3_7(400u, 300u);
        OutValActFL(400u, 300u);
        OutValActFR(400u, 300u);
        OutValActRL(400u, 300u);
        OutValActRR(400u, 300u);
        OutValActXL(400u, 300u);
        OutValActXR(400u, 300u);
        OutValActX3_10(4000u, 3000u);
        InValActX4_16(400u, 300u);
        OutValActX2_16(400u, 300u);
        ASRValActX(400u, 300u);
    }
    /* ---- Phase 3: 100% DC ---- */
    else
    {
        InValActFL(500u, 500u);
        InValActFR(500u, 500u);
        InValActRL(500u, 500u);
        InValActRR(500u, 500u);
        InValActXL(500u, 500u);
        InValActXR(500u, 500u);
        InValActX3_7(500u, 500u);
        OutValActFL(500u, 500u);
        OutValActFR(500u, 500u);
        OutValActRL(500u, 500u);
        OutValActRR(500u, 500u);
        OutValActXL(500u, 500u);
        OutValActXR(500u, 500u);
        OutValActX3_10(5000u, 5000u);
        InValActX4_16(500u, 500u);
        OutValActX2_16(500u, 500u);
        ASRValActX(500u, 500u);
    }
}

/* ===== 92 — ABS 阀故障上报 (FL/FR/RL/RR 进/排, 每字节 bit0=开路 bit1=短路) ===== */
void _92_PWM_CMP_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = PSWErrInValOpenFL  | (PSWErrInValShortFL  << 1);
    data[1] = PSWErrOutValOpenFL | (PSWErrOutValShortFL << 1);
    data[2] = PSWErrInValOpenFR  | (PSWErrInValShortFR  << 1);
    data[3] = PSWErrOutValOpenFR | (PSWErrOutValShortFR << 1);
    data[4] = PSWErrInValOpenRL  | (PSWErrInValShortRL  << 1);
    data[5] = PSWErrOutValOpenRL | (PSWErrOutValShortRL << 1);
    data[6] = PSWErrInValOpenRR  | (PSWErrInValShortRR  << 1);
    data[7] = PSWErrOutValOpenRR | (PSWErrOutValShortRR << 1);
    canPSWDbgMsgSend(0x92u, data, 8u);
}

/* ===== 93 — 21口 + 辅桥阀故障上报 (21口进/排, XL/XR 进/排) ===== */
void _93_PWM_CMP_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = PSWErr21InValveOpen  | (PSWErr21InValveShort  << 1);
    data[1] = PSWErr21OutValveOpen | (PSWErr21OutValveShort << 1);
    data[2] = PSWErrInValOpenXL    | (PSWErrInValShortXL    << 1);
    data[3] = PSWErrOutValOpenXL   | (PSWErrOutValShortXL   << 1);
    data[4] = PSWErrInValOpenXR    | (PSWErrInValShortXR    << 1);
    data[5] = PSWErrOutValOpenXR   | (PSWErrOutValShortXR   << 1);
    canPSWDbgMsgSend(0x93u, data, 6u);
}

/* ===== 94 — 22口 + TR_ASR 阀故障上报 (22口进/排, TR_ASR) ===== */
void _94_PWM_CMP_testCase(void)
{
    uint8_t data[8] = {0};
    data[0] = PSWErr22InValveOpen  | (PSWErr22InValveShort  << 1);
    data[1] = PSWErr22OutValveOpen | (PSWErr22OutValveShort << 1);
    data[2] = PSWErrASRValveOpenX  | (PSWErrASRValveShortX  << 1);
    canPSWDbgMsgSend(0x94u, data, 3u);
}

/* ===== A0 — 驱动芯片 U6/U9/U12/U13 开路/短路 =====
 * CAN 0xA0: [开路 U6,U9,U12,U13][短路 U6,U9,U12,U13] */
void _A0_PWM_CMP_testCase(void)
{
    uint8_t msg[8] = {0};
    msg[0] = PSWErrOpenDrive724_U6;
    msg[1] = PSWErrOpenDrive724_U9;
    msg[2] = PSWErrOpenDrive724_U12;
    msg[3] = PSWErrOpenDrive724_U13;
    msg[4] = PSWErrShortDrive724_U6;
    msg[5] = PSWErrShortDrive724_U9;
    msg[6] = PSWErrShortDrive724_U12;
    msg[7] = PSWErrShortDrive724_U13;
    canPSWDbgMsgSend(0xA0u, msg, 8u);
}

/* ===== A1 — 驱动芯片 U19 开路/短路 + 芯片总故障 =====
 * CAN 0xA1: [U19 开路, U19 短路, PSWErrDriveChip] */
void _A1_PWM_CMP_testCase(void)
{
    uint8_t msg[8] = {0};
    msg[0] = PSWErrOpenDrive724_U19;
    msg[1] = PSWErrShortDrive724_U19;
    msg[2] = PSWErrDriveChip;
    canPSWDbgMsgSend(0xA1u, msg, 3u);
}

/* ======================================================================== */
/*  测试用例注册表                                                             */
/* ======================================================================== */

static const debug_list_t g_debugCase_list[] =
{
    /* ---- 0x 通信 ---- */
    {0x00, _00_CAN0_testCase,           0, {}},
    {0x01, _01_UART_testCase,           0, {}},
    {0x02, _02_CAN_Filter_testCase,     1, {0x02}},
    {0x03, _03_CAN_busoff_testCase,     1, {0x03}},

    /* ---- 1x 系统 ---- */
    {0x10, _10_softReset_testCase,      1, {0x10}},
    {0x11, _11_softReset_testCase,      1, {0x11}},

    /* ---- 2x 看门狗 ---- */
    {0x20, _20_WD_testCase,             1, {0x20}},
    {0x21, _21_WD_testCase,             1, {0x21}},

    /* ---- 3x 定时器 ---- */
    {0x30, _30_10MStimer_testCase,      1, {0x30}},

    /* ---- 4x 电源/GPIO ---- */
    {0x40, _40_GPO_testCase,            1, {0x40}},
    {0x41, _41_GPI_testCase,            1, {0x41}},
    {0x42, _42_GPI_testCase,            1, {0x42}},

    /* ---- 6x EEPROM ---- */
    {0x60, _60_EEPROM_testCase,         1, {0x60}},

    /* ---- 7x 传感器采集 ---- */
    {0x70, _70_ADC_testCase,            1, {0x70}},
    {0x71, _71_ADC_testCase,            1, {0x71}},
    {0x72, _72_ADC_testCase,            1, {0x72}},

    /* ---- 8x 轮速计算值 ---- */
    {0x80, _80_PWM_testCase,            1, {0x80}},
    {0x81, _81_PWM_testCase,            1, {0x81}},

    /* ---- 9x/Ax 阀与芯片诊断 ---- */
    {0x90, _90_PWM_CMP_testCase,        1, {0x90}},
    {0x91, _91_PWM_CMP_testCase,        0, {}},
    {0x92, _92_PWM_CMP_testCase,        1, {0x92}},
    {0x93, _93_PWM_CMP_testCase,        1, {0x93}},
    {0x94, _94_PWM_CMP_testCase,        1, {0x94}},
    {0xA0, _A0_PWM_CMP_testCase,        1, {0xA0}},
    {0xA1, _A1_PWM_CMP_testCase,        1, {0xA1}},
};

static const uint8_t g_debug_case_num =
    (uint8_t)(sizeof(g_debugCase_list) / sizeof(g_debugCase_list[0]));

void PSWdebug(void)
{
    uint8_t i;

    /* ---- 注册新测试 ---- */
    if (RTEfDbgMsgSW3 != 0xFF)
    {
        if (g_debugOpenCnt < MAX_DEBUG_CNT)
        {
            /* 查重 */
            for (i = 0u; i < g_debugOpenCnt; i++)
            {
                if (g_debugOpenIndx[i] == RTEfDbgMsgSW3)
                {
                    break;
                }
            }
            if (i >= g_debugOpenCnt)
            {
                /* 匹配注册表 */
                for (uint8_t j = 0u; j < g_debug_case_num; j++)
                {
                    if (g_debugCase_list[j].debugIndx == RTEfDbgMsgSW3)
                    {
                        g_debugFuncList[g_debugOpenCnt] = g_debugCase_list[j].debugFunc;
                        g_debugOpenIndx[g_debugOpenCnt] = RTEfDbgMsgSW3;
                        g_debugOpenCnt++;
                        break;
                    }
                }
            }
        }
        RTEfDbgMsgSW3 = 0xFF;  /* 消费激活信号 */
    }

    /* ---- 执行已注册的测试 ---- */
    for (uint8_t j = 0u; j < g_debugOpenCnt; j++)
    {
        if (g_debugFuncList[j] != NULL)
        {
            g_debugFuncList[j]();
        }
    }
}
