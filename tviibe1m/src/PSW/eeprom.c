/**
 * @file    eeprom.c
 * @brief   M95040 EEPROM 命令层 — 三层 API (Low-Level → Block → RTE)
 */

#include <stddef.h>
#include "spi_eeprom.h"
#include "eeprom.h"
#include "wd_feed.h"
#include "psw_data.h"

/* ========================================================================== */
/*  命令码 & 状态寄存器位                                                       */
/* ========================================================================== */

#define CMD_WREN    0x06u
#define CMD_WRDI    0x04u
#define CMD_RDSR    0x05u
#define CMD_WRSR    0x01u
#define CMD_READ    0x03u       /* A8=0; A8=1 时需 | 0x08 */
#define CMD_WRITE   0x02u       /* 同上 */
#define A8_MASK     0x08u       /* 指令字节 bit3 = A8 */

#define SR_WIP      0x01u       /* Write In Progress */
#define SR_WEL      0x02u       /* Write Enable Latch */
#define SR_BP0      0x04u       /* Block Protect 0 */
#define SR_BP1      0x08u       /* Block Protect 1 */

#define VERIFY_COUNT 3u         /* 写后连续读回 N 次全匹配才确认成功 */

/* ========================================================================== */
/*  内部辅助 (static)                                                          */
/* ========================================================================== */

static void write_enable(void)
{
    uint8_t tx = CMD_WREN, rx;
    SpiEeprom_Transfer(&tx, &rx, 1u);
}

static uint8_t read_status(void)
{
    uint8_t tx[2] = { CMD_RDSR, 0x00u }, rx[2];

    /* 传输超时 → 返回 0x02 (WEL=1, WIP=0): 与老工程一致,
     * 保证 WEL/WIP 轮询在通信异常时也能退出, 不会卡死主循环 */
    if (!SpiEeprom_Transfer(tx, rx, 2u)) return 0x02u;

    return rx[1];
}

static bool is_busy(void)
{
    return (read_status() & SR_WIP) != 0u;
}

static uint8_t read_byte_cmd(uint16_t addr)
{
    uint8_t cmd = CMD_READ;
    if (addr & 0x0100u) cmd |= A8_MASK;
    return cmd;
}

static uint8_t write_byte_cmd(uint16_t addr)
{
    uint8_t cmd = CMD_WRITE;
    if (addr & 0x0100u) cmd |= A8_MASK;
    return cmd;
}

/** @brief 连续读回 N 次, 全部匹配才返回 true */
static bool verify_byte(uint16_t addr, uint8_t expected)
{
    for (uint8_t i = 0u; i < VERIFY_COUNT; i++)
    {
        if (Eeprom_ReadByte(addr) != expected) return false;
    }
    return true;
}

/** @brief WREN + 确认 WEL */
static bool enable_and_check(void)
{
    write_enable();
    return (read_status() & SR_WEL) != 0u;
}

/* ========================================================================== */
/*  Low-Level API — 原始字节/页操作                                             */
/* ========================================================================== */

void Eeprom_Init(void)
{
    uint8_t sr = read_status();
    if ((sr & (SR_BP1 | SR_BP0)) == 0u) return;  /* 无块保护, 跳过 */

    write_enable();
    if ((read_status() & SR_WEL) == 0u) return;

    uint8_t tx[2] = { CMD_WRSR, 0xF0u }, rx[2];  /* b7-b4=1, BP1:BP0=00 */
    SpiEeprom_Transfer(tx, rx, 2u);
    while (is_busy()) { wd_feed(); }
}

uint8_t Eeprom_ReadByte(uint16_t addr)
{
    uint8_t tx[3] = { read_byte_cmd(addr), (uint8_t)addr, 0x00u }, rx[3];
    if (!SpiEeprom_Transfer(tx, rx, 3u)) return 0u;   /* 超时已置故障标志 */
    return rx[2];
}

bool Eeprom_WriteByte(uint16_t addr, uint8_t data)
{
    if (!enable_and_check()) return false;

    uint8_t tx[3] = { write_byte_cmd(addr), (uint8_t)addr, data }, rx[3];
    if (!SpiEeprom_Transfer(tx, rx, 3u)) return false;
    while (is_busy()) { wd_feed(); }
    return verify_byte(addr, data);
}

void Eeprom_ReadSeq(uint16_t addr, uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0u; i < len; i++)
    {
        if (PSWfErrEEprom != 0u) { buf[i] = 0u; continue; }   /* 熔断后不再发事务 */
        buf[i] = Eeprom_ReadByte((uint16_t)(addr + i));
    }
}

bool Eeprom_WritePage(uint16_t addr, const uint8_t *data, uint8_t len)
{
    if (len == 0u || len > EEPROM_PAGE_SIZE) return false;
    /* M95040 页写超出页尾会在页内回卷覆盖 → 禁止跨页 (起点页内偏移 + 长度 ≤ 16) */
    if (((uint16_t)(addr & 0x0Fu) + (uint16_t)len) > EEPROM_PAGE_SIZE) return false;
    if (!enable_and_check()) return false;

    uint8_t tx[2u + EEPROM_PAGE_SIZE], rx[2u + EEPROM_PAGE_SIZE];
    tx[0] = write_byte_cmd(addr);
    tx[1] = (uint8_t)addr;
    for (uint8_t i = 0u; i < len; i++) tx[2u + i] = data[i];

    if (!SpiEeprom_Transfer(tx, rx, (uint32_t)(2u + len))) return false;
    while (is_busy()) { wd_feed(); }

    for (uint8_t i = 0u; i < len; i++)
    {
        if (!verify_byte((uint16_t)(addr + i), data[i])) return false;
    }
    return true;
}

/* ========================================================================== */
/*  Block API — 块级读写 (跨页 + 3x 校验 + 参数检查)                            */
/* ========================================================================== */

uint32_t Eeprom_WriteBlock(uint16_t addr, const uint8_t *data, uint32_t len)
{
    if (len == 0u || data == NULL) return 0u;
    if ((uint32_t)addr + len > EEPROM_TOTAL_SIZE) return 0u;   /* 容量越界 (同老工程 Eep_Write) */

    uint32_t written = 0u;
    while (written < len)
    {
        if (PSWfErrEEprom != 0u) return 0u;   /* 已熔断 → 中止 (同老工程) */
        uint16_t a       = (uint16_t)(addr + written);
        uint32_t remain  = len - written;
        uint8_t  pageOff = (uint8_t)(a & 0x0Fu);
        uint8_t  chunk   = (uint8_t)(EEPROM_PAGE_SIZE - pageOff);
        if (chunk > remain) chunk = (uint8_t)remain;

        if (!Eeprom_WritePage(a, &data[written], chunk))
            return 0u;
        written += chunk;
    }

    /* 3 次全量读回校验 */
    for (uint8_t pass = 0u; pass < 3u; pass++)
    {
        for (uint32_t i = 0u; i < len; i++)
        {
            if (Eeprom_ReadByte((uint16_t)(addr + i)) != data[i])
                return 0u;
        }
    }
    return len;
}

uint32_t Eeprom_ReadBlock(uint16_t addr, uint8_t *buf, uint32_t len)
{
    if (len == 0u || buf == NULL) return 0u;
    if ((uint32_t)addr + len > EEPROM_TOTAL_SIZE) return 0u;   /* 容量越界 (同老工程 Eep_Read) */
    Eeprom_ReadSeq(addr, buf, (uint16_t)len);
    if (PSWfErrEEprom != 0u) return 0u;   /* 熔断 → 报失败 (同老工程) */
    return len;
}

/* ========================================================================== */
/*  RTE API — 带故障上报 (供 rte_psw.c 薄包装)                                  */
/* ========================================================================== */

uint32_t Eeprom_Write(uint16_t eeBase, uint32_t relAddr,
                      const uint8_t *data, uint32_t len, uint32_t maxSize)
{
    if (relAddr + len > maxSize) { PSWfErrEEpromOutBnd = 1u; return 0u; }
    if (PSWfErrEEprom != 0u)       return 0u;

    uint16_t eeAddr = (uint16_t)(eeBase + relAddr);
    uint32_t result = Eeprom_WriteBlock(eeAddr, data, len);
    if (result == 0u) PSWfErrEEprom = 1u;
    return result;
}

uint32_t Eeprom_Read(uint16_t eeBase, uint32_t relAddr,
                     uint8_t *buf, uint32_t len, uint32_t maxSize)
{
    if (relAddr + len > maxSize) { PSWfErrEEpromOutBnd = 1u; return 0u; }
    if (PSWfErrEEprom != 0u)       return 0u;
    return Eeprom_ReadBlock((uint16_t)(eeBase + relAddr), buf, len);
}
