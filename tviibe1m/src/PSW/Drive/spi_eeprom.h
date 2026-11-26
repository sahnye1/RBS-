/**
 * @file    spi_eeprom.h
 * @brief   SCB3 SPI 主模式传输层 — 为 M95040 EEPROM 提供底层通信
 *
 * @details 硬件: P13.0=MISO, P13.1=MOSI, P13.2=SCK, P13.3=CS(SS0)
 *          主模式轮询, 无需 NVIC 中断。2MHz 波特率。
 */

#ifndef SPI_EEPROM_H
#define SPI_EEPROM_H

#include <stdint.h>
#include <stdbool.h>

void SpiEeprom_Init(void);

/** @brief 阻塞式全双工传输 (主模式轮询)
 *  @return true = 传输完成; false = 超时 (已置 PSWfErrEEprom 熔断, 调用方须中止本次操作) */
bool SpiEeprom_Transfer(uint8_t *tx, uint8_t *rx, uint32_t size);

#endif /* SPI_EEPROM_H */
