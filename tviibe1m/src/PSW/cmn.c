/**
 * @file    cmn.c
 * @brief   PSW 公共模块 — 工具函数 + 板级初始化 + 10ms 主处理
 *
 * @details 工具函数: periph_divider / delay_us / SoftWD_init / wd_feed /
 *          sysRstReason_get / system_update_start / system_reboot
 *          PSW_10ms_deal(): 10ms 主处理, 执行顺序见函数内 ①~⑩ 注释
 *            (⑪ 是 TestDeal_Late() 尾部的 10ms 节拍验证测试, 不在主序列里)
 *
 * @note    测试/调试代码已集中到 test/ 目录:
 *            test_cfg.h  — 全部开关 (总开关 TEST_DEAL_ENABLE + 分项)
 *            test_deal.c — 实现 (阀波形 / CAN 调试上报 / 节拍验证)
 *          本文件只保留 TestDeal_Early/Mid/Late 三个入口调用, 位置有顺序依赖。
 */

#include "cmn.h"
#include "can.h"
#include "wheel_speed.h"
#include "adc.h"
#include "bts724g.h"
#include "psw_data.h"
#include "RTE.h"
#include "spi_eeprom.h"
#include "eeprom.h"
#include "timer10ms.h"
#include "gpio.h"
#include "rte_psw.h"
#include "test_cfg.h"
#include "test_deal.h"

/* ======================================================================== */
/*  delay_us — 微秒延时                                                      */
/* ======================================================================== */
void delay_us(uint32_t n)
{
    Cy_SysTick_DelayInUs(n);
}

/* ======================================================================== */
/*  periph_divider — 外设时钟分频器                         */
/*  动态读取 clk_peri 实际频率, 按目标频率自动计算分频值, 不硬编码源频率。      */
/*  改标定/晶振后, 各外设时钟自动跟随, 无需手动改分频值。外部晶振8000000hz，通过倍频处理后cm4主频80000000hz，分到外设为40000000hz  */
/* ======================================================================== */
void periph_divider(en_clk_dst_t ipBlock, cy_en_divider_types_t div_type, uint32_t clk_no, uint32_t targetFreq)
{
    if (div_type >= CY_SYSCLK_DIV_TYPE_NUM)
    {
        return;
    }
    uint32_t divIntNum = 0;
    uint32_t divFractNum = 0;
    uint32_t periFreq = 0;
    Cy_SysClk_GetClkPeriFrequency(&periFreq);

    Cy_SysClk_PeriphAssignDivider(ipBlock, div_type, clk_no);

    if (div_type == CY_SYSCLK_DIV_8_BIT || div_type == CY_SYSCLK_DIV_16_BIT)
    {
        divIntNum = DIV_ROUND_UP(periFreq, targetFreq);
        Cy_SysClk_PeriphSetDivider(div_type, clk_no, (divIntNum - 1ul));
    }
    else
    {
        divIntNum = periFreq / targetFreq;
        /* 小数分频值 = (余数 / 目标频率) × 32, 四舍五入。
         * 用整数运算实现, 避免 (double) 拉入整套浮点数学库 (约 1.7KB flash)。 */
        divFractNum = (((periFreq % targetFreq) * 32ul) + (targetFreq / 2ul)) / targetFreq;
        Cy_SysClk_PeriphSetFracDivider(div_type, clk_no, (divIntNum - 1ul), divFractNum);
    }
    /* Enable the divider */
    Cy_SysClk_PeriphEnableDivider(div_type, clk_no);
}

/* ======================================================================== */
/*  SoftWD_init — 软件看门狗初始化                                             */
/* ======================================================================== */
void SoftWD_init(void)
{
    Cy_SysReset_ClearAllResetReasons();      /* 清除历史复位原因 */
    Cy_WDT_Init();
    Cy_WDT_Unlock();
    Cy_WDT_SetUpperLimit(2000ul);            /* 上限 2000ms */
    Cy_WDT_SetDebugRun(CY_WDT_ENABLE);       /* 调试模式下继续计数 */
    Cy_WDT_Lock();
    Cy_WDT_Enable();
}

/* ======================================================================== */
/*  看门狗跳过标记 (PSWdebug 测试用例动态控制)                                  */
/* ======================================================================== */
uint8_t g_skipSoftWD = 0u;
uint8_t g_skipHardWD = 0u;

/* ======================================================================== */
/*  wd_feed — 双看门狗喂狗                                                    */
/* ======================================================================== */
void wd_feed(void)
{
    if (g_skipHardWD == 0u)
    {
        Cy_GPIO_Inv(WD_HW_GPIO_PORT, WD_HW_GPIO_PIN);   /* 硬件看门狗: P5.0 翻转 */
    }
    if (g_skipSoftWD == 0u)
    {
        Cy_WDT_ClearWatchdog();                          /* 软件看门狗: WDT 清零 */
    }
}

/* ======================================================================== */
/*  sysRstReason_get — 复位原因查询 (1=WDT, 0=其他)                           */
/* ======================================================================== */
uint8_t sysRstReason_get(void)
{
    uint32_t resetReason;

    resetReason = Cy_SysReset_GetResetReason();
    Cy_SysReset_ClearAllResetReasons();

    if ((resetReason & CY_SYSRESET_WDT) != 0ul)
    {
        return 1;
    }
    return 0;
}

/* ======================================================================== */
/*  system_update_start — 系统升级入口                                        */
/* ======================================================================== */
void system_update_start(uint32_t updateFlag)
{
    CY_SET_REG32(RAM_1_ADDRESS, updateFlag);

    /* 等待 SRAM1 写入完成 */
    while (0ul == (Cy_Cpu_SramWriteBufferStatus(CY_CPU_SRAM1)))
    {
        ;
    }

    /* SRAM1 配置为保持模式 (复位后数据不丢失) */
    Cy_Cpu_SramPowerModeSet(CY_CPU_SRAM1, CY_CPU_SRAM_PM_RETAINED, 0ul);

    NVIC_SystemReset();
}

/* ======================================================================== */
/*  system_reboot — 系统软复位                                                */
/* ======================================================================== */
void system_reboot(void)
{
    NVIC_SystemReset();
}

/* ======================================================================== */
/*  CAN RX 过滤表 — 开发阶段硬编码占位, 正式版由 COM 层通过 RTE 接口下发          */
/*                                                                            */
/*  CAN0 = 公共CAN (底盘CAN, J1939) — 需按整车网络协议适配                       */
/*  CAN1 = 调试CAN (诊断/标定工具) — 诊断 ID                                   */
/* ======================================================================== */

/* ======================================================================== */
/*  board_init — 板级初始化 (含 GPIO/CAN/继电器)                              */
/*                                                                           */
/*  初始化步骤 (有先后依赖):                                                  */
/*   1) SystemInit + 开中断;  WDT → SoftWD_init                              */
/*   2) GPIO → Gpio_Init;  闭合总继电器 (P14.3) → VPOWER 上电                 */
/*   3) CAN → can_init (ID 白名单由上层 can_info_cfg 下发)                    */
/*   4) ADC → Adc_Init (SAR0 九通道一个组 + SAR1 P12.1)                       */
/*   5) 轮速 → WheelSpeed_Init;  阀驱动 → Bts724g_Init (含 ValvePwm_Init)     */
/*   6) EEPROM → SpiEeprom_Init + Eeprom_Init; 7) 定时器 → Timer10ms_Init     */
/* ======================================================================== */
void board_init(void)
{
    SystemInit();
    __enable_irq();

    /* PSW 版本号赋值 (RTEPSW_Version 定义在 rte_psw.c, 此处只赋值不定义) */
    RtePsw_VersionInit();

    /* WDT 初始化 */
    SoftWD_init();

    /* GPIO 引脚初始化 (继电器/UB/指示灯/接插件, 上电默认状态) */
    Gpio_Init();

    /* 闭合总继电器 (P14.3),*/
    Gpio_RelayCtrl(1u);

    /* CAN0 + CAN1 初始化 (公共CAN + 调试CAN, ID 白名单由 can_info_cfg 配置) */
    can_init();

    /* CAN 初始化耗时较长, 中间喂一次狗 */
    wd_feed();

    /* ADC: SAR0 (轮速诊断6 + 压力2 + VPOWER1 合成一个 group) + SAR1 (P12.1) */
    Adc_Init();

    /* 轮速采集 — 6 通道 TCPWM 捕获 */
    WheelSpeed_Init();

    /* 阀驱动 — BTS724G SPI 初始化 */
    Bts724g_Init();

    /* SPI EEPROM 传输层 + 命令层 */
    SpiEeprom_Init();
    Eeprom_Init();

    wd_feed();

    /* 10ms 周期定时器 (最后启动, ISR 开始产生节拍) */
    Timer10ms_Init();

    wd_feed();
}

/* ======================================================================== */
/*  PSW_10ms_deal — 10ms 周期主处理                                          */
/*                                                                           */
/*  各驱动 → 本层模块对应:                                                    */
/*  ADC 采集 (轮速诊断6 + 压力2 + VPOWER1 + P12.1) → adc                       */
/*  TCPWM 捕获 + PDMA 搬运 → wheel_speed                                     */
/*  阀/芯片电气诊断 → ValveDiag_Process (bts724g)                            */
/*  压力 + VPOWER 采集 → Adc_Read (adc)                                      */
/*                                                                           */
/*  测试/调试: TestDeal_Early/Mid/Late 三个入口 (开关见 test_cfg.h)            */
/*  主循环可简化为:                                                           */
/*    if (g_b10msTick) { PSW_10ms_deal(); }                                  */
/* ======================================================================== */
void PSW_10ms_deal(void)
{
    /* ① 喂狗 — 硬狗 (GPIO 翻转) + 软狗 (WDT 清零) */
    wd_feed();

    /* ② ADC 采集: SAR0 组 (CH0~CH8 = 轮速诊断6 + 压力2 + VPOWER) + P12.1
     *    (必须先于阀诊断: 阀诊断门控依赖 PSWvIgn ← VPOWER) */
    Adc_Read();

    /* ③ PSW* 中间变量刷新 (VPOWER 电压 → PSWvIgn, 阀诊断门控依赖) */
    PSWData_Refresh();

    /* ③.5~④ 测试: 阀保压阶梯标定 / 阀波形循环
     *        须晚于 PSWData_Refresh (压力已刷新), 早于 ValveDiag_Process
     *        (阀保压模块内部每拍 NotifyValveActive 让电气诊断让路) */
    TestDeal_Early();

    /* ④ 阀诊断状态机 + 芯片级故障汇总 (依赖 PSWvIgn 门控) */
    ValveDiag_Process();

    /* ⑤ 轮速计算 + 中值滤波 + CAN 发送 */
    WheelSpeed_CalcAndSend();

    /* ⑥ ADC 轮速传感器诊断 (开路/短路/间隙; 须在轮速计算之后 — 间隙判定用车速门控) */
    Adc_Process();

    /* ⑥ 测试: 阀诊断明细输出 (VALVE_DIAG_DEBUG) */
    TestDeal_Mid();

    /* ⑦ 继电器 VPOWER 判定节拍推进 (无 pending 时极速返回) */
    Gpio_RelayTick();

    /* ⑧ RTE 数据写入 (PSW* 已在步骤③刷新) */
    PSWDataToRte();

    /* ⑧.5~⑧.7 + ⑪ 测试: CAN 调试上报 (RTE 0x700 / PSW 0x710 / 压力 0x730)
     *               + 10ms 节拍翻转 P22.0   (须晚于 PSWDataToRte) */
    TestDeal_Late();

    /* ⑨ CAN0/CAN1 busoff 自动恢复 */
    can0_busoff_resume();
    can1_busoff_resume();

    /* ⑩ 清除 10ms 节拍标志 */
    g_b10msTick = false;
}
