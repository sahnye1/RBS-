/**
 * @file    eeprom.h
 * @brief   M95040 EEPROM 命令层 — 三层 API (Low-Level → Block → RTE)
 *
 * @details 依赖 spi_eeprom.h 提供底层 SPI 传输。
 *          W 引脚硬接 VCC, 状态寄存器可自由写入。
 *
 *          API 分层:
 *          │ 层            │ 函数                     │ 说明                     │
 *          │───────────────│─────────────────────────│─────────────────────────│
 *          │ Low-Level     │ Init / ReadByte / WriteByte / ReadSeq / WritePage │
 *          │ Block         │ ReadBlock / WriteBlock   │ 带校验/跨页/安全检查     │
 *          │ RTE           │ Read / Write             │ 含 PSWfErrEEprom 故障上报 │
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include <stdbool.h>

#define EEPROM_PAGE_SIZE    16u
#define EEPROM_TOTAL_SIZE   512u

/* ========================================================================== */
/*  分区布局                                                                    */
/* ========================================================================== */
/*     ERR_CODE 0x0000   128 B   |   ASW      0x0080   128 B                   */
/*     PSW      0x0100    32 B   |   COM      0x0120    32 B                   */
/*     BOOT     0x0140    32 B   |   保留     0x0160   160 B (未分配)           */
/*                                                                            */
/*  @note  各区起始与长度均为 16 B 页的整数倍 (不跨页); 已用 0x0000~0x015F      */
/*         各区相对地址上限 = 本区 SIZE, 越界即置 PSWfErrEEpromOutBnd          */
#define EEPROM_ERR_CODE_BASE     0x0000u
#define EEPROM_ERR_CODE_SIZE     128u
#define EEPROM_ASW_BASE          0x0080u
#define EEPROM_ASW_SIZE          128u
#define EEPROM_ASW_CFG_SIZE      128u    /* = ASW_SIZE */
#define EEPROM_PSW_BASE          0x0100u
#define EEPROM_PSW_SIZE          32u
#define EEPROM_COM_BASE          0x0120u
#define EEPROM_COM_SIZE          32u
#define EEPROM_BOOT_BASE         0x0140u
#define EEPROM_BOOT_SIZE         32u

/* ========================================================================== */
/*  Low-Level API — 原始字节/页操作                                              */
/* ========================================================================== */

void    Eeprom_Init(void);
uint8_t Eeprom_ReadByte(uint16_t addr);
bool    Eeprom_WriteByte(uint16_t addr, uint8_t data);
void    Eeprom_ReadSeq(uint16_t addr, uint8_t *buf, uint16_t len);
bool    Eeprom_WritePage(uint16_t addr, const uint8_t *data, uint8_t len);

/* ========================================================================== */
/*  Block API — 块级读写 (跨页 + 3x 校验 + 参数检查)                             */
/* ========================================================================== */

/** @brief 块写入: 自动跨页 + 写后 3 次读回校验 */
uint32_t Eeprom_WriteBlock(uint16_t addr, const uint8_t *data, uint32_t len);

/** @brief 块读取: NULL/len=0 安全检查 */
uint32_t Eeprom_ReadBlock(uint16_t addr, uint8_t *buf, uint32_t len);

/* ========================================================================== */
/*  RTE API — 供 rte_psw.c 薄包装, 含故障上报                                    */
/*                                                                            */
/*  越界检查 → PSWfErrEEpromOutBnd, 写入失败 → PSWfErrEEprom 熔断               */
/* ========================================================================== */

uint32_t Eeprom_Write(uint16_t eeBase, uint32_t relAddr,
                      const uint8_t *data, uint32_t len, uint32_t maxSize);
uint32_t Eeprom_Read(uint16_t eeBase, uint32_t relAddr,
                     uint8_t *buf, uint32_t len, uint32_t maxSize);

#endif /* EEPROM_H */
