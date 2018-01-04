//
// Created by kirill on 27.12.17.
//

#ifndef SDSPI_SD_H
#define SDSPI_SD_H

#include <stdint.h>
#include "stm32l1xx_hal.h"

// Chip select port/pin and helper wrappers
#define CS_SD_PORT GPIOB
#define CS_SD_PIN GPIO_PIN_12
#define SS_SD_SELECT() HAL_GPIO_WritePin(CS_SD_PORT, CS_SD_PIN, GPIO_PIN_RESET)
#define SS_SD_DESELECT() HAL_GPIO_WritePin(CS_SD_PORT, CS_SD_PIN, GPIO_PIN_SET)

// Card type flags (CardType)
#define CT_BASE 0
#define CT_MMC 0x01
#define CT_SD1 0x02
#define CT_SD2 0x04
#define CT_SDC (CT_SD1|CT_SD2)
#define CT_BLOCK 0x08

typedef struct {
  __IO int8_t type;
} SD_Info;

typedef enum {
  SD_OK,
  SD_ERROR
} SD_StatusDef;

SD_StatusDef SD_Init(void);
void SPI_Release(void);
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba);
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba);

#endif // SDSPI_SD_H
