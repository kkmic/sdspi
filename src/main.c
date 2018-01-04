/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c
  * @author  MCD Application Team
  * @brief   This example shows how to retarget the C library printf function
  *          to the UART.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//#include <string.h>
#include <ff.h>
#include <stdlib.h>
#include "main.h"
#include "fatfs.h"
#include "sd.h"

/* Global structures and functions */
UART_HandleTypeDef UartHandle;
SPI_HandleTypeDef hspi2;

void Error_Handler(const char* message, int8_t res);

/* Private structures ---------------------------------------------------------*/
static FATFS SDFatFs;
static char USERPath[4];   /* USER logical drive path */
static FIL USERFile;  /* File object for USER */

static uint8_t sect[512];

/* Private functions ---------------------------------------------------------*/
static void SystemClock_Config(void);
static void UART_Init(void);
static void SPI_Init(void);
static FRESULT ReadLongFile(uint16_t limit);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L1xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  /* Configure the system clock to 32 MHz */
  UART_Init();
  SystemClock_Config();
  SPI_Init();

  BSP_LED_On(LED3);
  BSP_LED_On(LED4);
  HAL_Delay(200);
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  HAL_Delay(200);

  printf("\r\nInit ready.\r\n");

  if (FATFS_LinkDriver(&USER_Driver, USERPath) != 0) {
    Error_Handler("Init failed", 0);

  } else {
    printf("FatFS link driver successful\r\n");
    FRESULT res;
    if ((res = f_mount(&SDFatFs, USERPath, 0)) != FR_OK) {
      Error_Handler("Mount failed", res);
    } else {
      if ((res = f_open(&USERFile, "test/test.mp4", FA_READ)) != FR_OK) {
        Error_Handler("Open failed", res);
      } else {
        ReadLongFile(128); // buffers
        printf("Done.\r\n");

        if ((res = f_close(&USERFile)) != FR_OK) {
          Error_Handler("Close failed", res);
        }
        BSP_LED_On(LED3);
        for (;;) {}
      }
    }
  }
}

//    DWORD fre_clust, fre_sect, tot_sect;
//    FILINFO fileInfo;
//
//    fileInfo.lfname = (char*)sect;
//    fileInfo.lfsize = sizeof(sect);
//    FRESULT result = f_opendir(&dir, "/");
//    if (result == FR_OK)
//    {
//      while(1)
//      {
//        result = f_readdir(&dir, &fileInfo);
//        if (result==FR_OK && fileInfo.fname[0])
//        {
//          fn = fileInfo.lfname;
//          if(strlen(fn)) HAL_UART_Transmit(&huart1,(uint8_t*)fn,strlen(fn),0x1000);
//          else HAL_UART_Transmit(&huart1,(uint8_t*)fileInfo.fname,strlen((char*)fileInfo.fname),0x1000);
//          if(fileInfo.fattrib&AM_DIR)
//          {
//            HAL_UART_Transmit(&huart1,(uint8_t*)"  [DIR]",7,0x1000);
//          }
//        }
//        else break;
//        HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
//      }
//      f_closedir(&dir);
//    }
//  if (SD_Init() == SD_OK) {
//    BSP_LED_On(LED3);
//
//  } else {
//    printf("Can't init SD card\r\n");
//    Error_Handler();
//  }

static void UART_Init(void)
{
/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit)
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 230400;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(NULL, 0);
  }
}

static void SPI_Init(void)
{
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;

  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler("Can't init SPI module", 0);
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __io_putchar(int ch)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

#ifdef MEMORY_DEBUG
static const char DIGIT[] = "0123456789abcdef";

void print_address(caddr_t addr) {
  uint32_t av = (uint32_t)addr;

  for(int i = 3; i >=0; i--) {
    uint8_t ab = (av >> i*8);

    __io_putchar(DIGIT[(ab & 0xf0) >> 4]);
    __io_putchar(DIGIT[ab & 0x0f]);
  }
}
#endif

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 16000000
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 6
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(NULL, 0);
  }

  /* Set Voltage scale1 as MCU will run at 32MHz */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler(NULL, 0);
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(const char* message, int8_t res)
{
  if (message != NULL) {
    printf("%s: Res = %d\r\n", message, res);
  }

  /* Turn LED3 on */
  BSP_LED_On(LED4);
  for(;;) {}
}

FRESULT ReadLongFile(uint16_t limit)
{
  uint16_t step=0;
  UINT bytesRead;
  uint32_t f_size = USERFile.fsize;

  printf("File size: %lu\r\n", f_size);

  for (uint32_t index = 0, count = 0; f_size > 0 && count < limit; index += step, count++) {
    step = f_size < 512 ? f_size : 512;
    f_size -= step;

    f_lseek(&USERFile, index);
    f_read (&USERFile, sect, step, &bytesRead);
  }
  return FR_OK;
}
