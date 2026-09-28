/**
 * @file    spi_eeprom.c
 * @brief   SCB3 SPI 主模式传输层实现 (Low-Level API, 纯轮询, 无中断)
 *
 * @details SCB3 主模式, 2MHz (40MHz ÷2.5 → 16MHz SCB ÷8 过采样)。
 *          P13.0=MISO, P13.1=MOSI, P13.2=SCK, P13.3=CS(SS0)。
 *          Low-Level API 不依赖中断和 context, 直接操作 FIFO + 状态轮询。
 *
 * @note    High-Level API (Cy_SCB_SPI_Transfer) 需要 ISR 调用
 *          Cy_SCB_SPI_Interrupt() 来搬运 FIFO 数据。
 *          本模块选用 Low-Level API 的真实理由（2026-09-28 按代码核实订正）：
 *            ① SPI 传输很短、且在 10ms 任务里同步调用, 全阻塞实现最简单、时序确定;
 *            ② 不必额外占用中断资源 —— 本工程实际只用了 CPUIntIdx1~3
 *               (CAN0=CAN_CHSS_IRQ_INDX / CAN1=CAN_PRVT_IRQ_INDX / 10ms 定时器=CPUIntIdx3,
 *                见 can.h、timer10ms.c), **并非"8 个槽已满"**, 另有空闲中断线可用;
 *            ③ 无需 context (Cy_SCB_SPI_Init(..., NULL)), 不存在重入问题。
 */

#include "cy_project.h"
#include "cy_device_headers.h"
#include "spi_eeprom.h"
#include "wd_feed.h"
#include "cmn.h"
#include "psw_data.h"       /* PSWfErrEEprom: SPI 超时熔断标志 */

/* ========================================================================== */
/*  硬件宏                                                                      */
/* ========================================================================== */
#define EEP_SCB                 SCB3
#define EEP_PCLK                PCLK_SCB3_CLOCK
#define EEP_SS_IDX              CY_SCB_SPI_SLAVE_SELECT0

/* 2MHz 波特率: SCB 时钟 16MHz (动态分频) / 8 过采样 = 2MHz */
#define SCB_SPI_BAUDRATE        2000000ul
#define SCB_SPI_OVERSAMPLING    8ul
#define SCB_SPI_CLOCK_FREQ      (SCB_SPI_BAUDRATE * SCB_SPI_OVERSAMPLING)
#define CLK_DIV_NO              1u

/* P13.x → SCB3 HSIOM (均为值 19) */
#define P13_MISO_HSIOM          P13_0_SCB3_SPI_MISO
#define P13_MOSI_HSIOM          P13_1_SCB3_SPI_MOSI
#define P13_SCK_HSIOM           P13_2_SCB3_SPI_CLK
#define P13_CS_HSIOM            P13_3_SCB3_SPI_SELECT0

/* SPI 传输超时 (防止硬件异常导致死循环) */
#define SPI_XFER_TIMEOUT        0x100000ul

/* ========================================================================== */
/*  GPIO 引脚配置                                                               */
/* ========================================================================== */

static const cy_stc_gpio_pin_prt_config_t spi_eeprom_pin_cfg[] =
{
    { GPIO_PRT13, 0u, 0ul, CY_GPIO_DM_HIGHZ,  P13_MISO_HSIOM, 0ul, 0ul, 0ul, 0ul, 0ul },
    { GPIO_PRT13, 1u, 0ul, CY_GPIO_DM_STRONG, P13_MOSI_HSIOM, 0ul, 0ul, 0ul, 0ul, 0ul },
    { GPIO_PRT13, 2u, 0ul, CY_GPIO_DM_STRONG, P13_SCK_HSIOM,  0ul, 0ul, 0ul, 0ul, 0ul },
    { GPIO_PRT13, 3u, 1ul, CY_GPIO_DM_STRONG, P13_CS_HSIOM,   0ul, 0ul, 0ul, 0ul, 0ul },
};
#define SPI_EEPROM_PIN_NUM (sizeof(spi_eeprom_pin_cfg) / sizeof(spi_eeprom_pin_cfg[0]))

/* ========================================================================== */
/*  公开 API                                                                    */
/* ========================================================================== */

void SpiEeprom_Init(void)
{
    /* 1. GPIO 初始化: P13.0-3 → SCB3 复用功能 */
    Cy_GPIO_Multi_Pin_Init(spi_eeprom_pin_cfg, SPI_EEPROM_PIN_NUM);

    /* 2. 时钟: PCLK_SCB3 → DIV_24_5#1 → 16MHz (periph_divider 动态) */
    periph_divider(EEP_PCLK, CY_SYSCLK_DIV_24_5_BIT, CLK_DIV_NO, SCB_SPI_CLOCK_FREQ);

    /* 3. SPI 主模式, Low-Level API (context = NULL, 不占中断) */
    Cy_SCB_SPI_DeInit(EEP_SCB);

    cy_stc_scb_spi_config_t spiCfg =
    {
        .spiMode                    = CY_SCB_SPI_MASTER,
        .subMode                    = CY_SCB_SPI_MOTOROLA,
        .sclkMode                   = CY_SCB_SPI_CPHA0_CPOL0,
        .oversample                 = SCB_SPI_OVERSAMPLING,
        .rxDataWidth                = 8u,
        .txDataWidth                = 8u,
        .enableMsbFirst             = true,
        .enableFreeRunSclk          = false,
        .enableInputFilter          = false,
        .enableMisoLateSample       = true,
        .enableTransferSeperation   = false,
        .ssPolarity0                = CY_SCB_SPI_ACTIVE_LOW,
        .ssPolarity1                = CY_SCB_SPI_ACTIVE_LOW,
        .ssPolarity2                = CY_SCB_SPI_ACTIVE_LOW,
        .ssPolarity3                = CY_SCB_SPI_ACTIVE_LOW,
        .enableWakeFromSleep        = false,
        .txFifoTriggerLevel         = 0u,
        .rxFifoTriggerLevel         = 1u,
        .rxFifoIntEnableMask        = 0u,
        .txFifoIntEnableMask        = 0u,
        .masterSlaveIntEnableMask   = 0u,
        .enableSpiDoneInterrupt     = false,
        .enableSpiBusErrorInterrupt = false,
    };

    Cy_SCB_SPI_Init(EEP_SCB, &spiCfg, NULL);
    Cy_SCB_SPI_SetActiveSlaveSelect(EEP_SCB, EEP_SS_IDX);
    Cy_SCB_SPI_Enable(EEP_SCB);
}

bool SpiEeprom_Transfer(uint8_t *tx, uint8_t *rx, uint32_t size)
{
    volatile uint32_t timeout;

    /* 0. 清空 RX FIFO, 避免上一次传输的残留数据污染本次结果 */
    Cy_SCB_SPI_ClearRxFifo(EEP_SCB);

    /* 1. 将所有 TX 数据推送进 FIFO (主模式: 写入即启动传输) */
    Cy_SCB_SPI_WriteArrayBlocking(EEP_SCB, tx, size);

    /* 2. 等待 TX 完成 (TX FIFO + 移位寄存器全空) */
    timeout = SPI_XFER_TIMEOUT;
    while (!Cy_SCB_SPI_IsTxComplete(EEP_SCB))
    {
        wd_feed();
        if (--timeout == 0u)
        {
            PSWfErrEEprom = 1u;   /* 熔断: 后续 EEPROM 操作直接返回失败 (同老工程) */
            return false;
        }
    }

    /* 3. 确保 RX FIFO 已收齐 size 个字节 (全双工: TX 完成时 RX 通常也完成,
     *    但增加等待确保时序安全) */
    timeout = SPI_XFER_TIMEOUT;
    while (Cy_SCB_SPI_GetNumInRxFifo(EEP_SCB) < size)
    {
        wd_feed();
        if (--timeout == 0u)
        {
            PSWfErrEEprom = 1u;
            return false;
        }
    }

    /* 4. 从 RX FIFO 读出全部响应数据 */
    Cy_SCB_SPI_ReadArray(EEP_SCB, rx, size);
    return true;
}

/* [] END OF FILE */
