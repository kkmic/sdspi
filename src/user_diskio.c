/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future. 
 * Kept to ensure backward compatibility with previous CubeMx versions when 
 * migrating projects. 
 * User code previously added there should be copied in the new user sections before 
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <sd.h>
#include "ff_gen_drv.h"
#include "sd.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
extern SD_Info sd_info;

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize(BYTE pdrv);

DSTATUS USER_status(BYTE pdrv);

DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);

#if _USE_WRITE == 1

DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);

#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1

DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff);

#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef USER_Driver =
    {
        USER_initialize,
        USER_status,
        USER_read,
#if  _USE_WRITE
        USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
        USER_ioctl,
#endif /* _USE_IOCTL == 1 */
    };

/* Private functions ---------------------------------------------------------*/
//static inline const char* getCardTypeName() {
//  if (sd_info.type == CT_BASE) {
//    return "Base";
//  } else if (sd_info.type == CT_MMC) {
//    return "MMC";
//  } else if (sd_info.type == (CT_SD2 | CT_BLOCK)) {
//    return "SDHC";
//  } else if (sd_info.type == (CT_SD2)) {
//    return "SDSC";
//  } else if (sd_info.type == CT_SD1) {
//    return "SDv1";
//  } else {
//    return "Unknown";
//  }
//}

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (BYTE pdrv)  // Physical drive number to identify the drive
{
  Stat = STA_NOINIT;

//  printf("USER_initialize: %d\r\n", pdrv);
  if (SD_Init() == 0) {
    Stat &= ~STA_NOINIT;
//    printf("Card type: %s\r\n", getCardTypeName());
  }

  return Stat;
}

/**
  * @brief  Gets Disk Status 
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (BYTE pdrv)  // Physical drive number to identify the drive
{
//  printf("USER_status: %d\r\n", pdrv);

  if (pdrv > 0) {
    return STA_NOINIT;
  }

  return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s) 
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read(
    BYTE pdrv,      /* Physical drive number to identify the drive */
    BYTE *buff,     /* Data buffer to store read data */
    DWORD sector,   /* Sector address in LBA */
    UINT count)     /* Number of sectors to read */
{
//  printf("USER_read: %d, sector=%ld, count=%d\r\n", pdrv, sector, count);

  if (pdrv > 0 || count != 1) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  if ((sd_info.type & CT_SD2) == 0) {
    sector *= 512; // Convert to byte address if not SDv2 card type
  }

  SD_Read_Block(buff, sector); // Read in a buffer
  SPI_Release();
  return RES_OK;
}

/**
  * @brief  Writes Sector(s)  
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1

DRESULT USER_write(
    BYTE pdrv,          /* Physical drive number to identify the drive */
    const BYTE *buff,   /* Data to be written */
    DWORD sector,       /* Sector address in LBA */
    UINT count          /* Number of sectors to write */
)
{
//  printf("USER_write: %d, sector=%ld, count=%d\r\n", pdrv, sector, count);

  if (pdrv || count != 1) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;
  if (Stat & STA_PROTECT) return RES_WRPRT;

  if (!(sd_info.type & 4)) sector *= 512; /* Convert to byte address if needed */
  SD_Write_Block((BYTE*)buff,sector); //Считаем блок в буфер

  SPI_Release();

  return RES_OK;
}

#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation  
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1

DRESULT USER_ioctl(
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
//  printf("USER_ioctl: %d, cmd=%d\r\n", pdrv, cmd);

  if (pdrv > 0) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  DRESULT res = RES_PARERR;
  switch (cmd)
  {
    case CTRL_SYNC : /* Flush dirty buffer if present */
      SS_SD_SELECT();
      if (SPI_wait_ready() == 0xFF)
        res = RES_OK;
      break;

    case GET_SECTOR_SIZE : /* Get sectors on the disk (WORD) */
      *(WORD*)buff = 512;
      res = RES_OK;
      break;

    default:
      break;
  }

  SPI_Release();
  return res;
}

#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
