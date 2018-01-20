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
#include <string.h>
#include "main.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "user_diskio.h" /* defines USER_Driver as external */
#include "sd.h"

/* Global structures and functions */
UART_HandleTypeDef UartHandle;
SPI_HandleTypeDef hspi2;

/* Variables to manage push button on board: interface between ExtLine interruption and main program */
__IO uint8_t ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */
//__IO uint8_t resetReason = UNDEFINED;

/* Private structures ---------------------------------------------------------*/
static FIL USERFile;
static FATFS SDFatFs;
static char USERPath[4];   /* USER logical drive path */

//static uint8_t sect[512];

#define STR_LENGTH 64
char str[STR_LENGTH];
void uart_print(int strlen);

void Error_Handler(const char* message, int8_t res);

/* Private functions ---------------------------------------------------------*/
static void SystemClock_Config(void);
static void UART_Init(void);
static void SPI_Init(void);

#define ADC_BUFFER_LENGTH 200
static uint16_t adcBuffer[ADC_BUFFER_LENGTH];

#define MAX_FILE_SIZE (200 * 1024 * 1024)

static DWORD getFreeSize();
static int getLastFileIndex();

static const char DIRECTORY[] = "/TEST";
static const char FILE_PREFIX[] = "OSC_";
static const char FILE_EXTENSION[] = ".DAT";

static const size_t FILE_PREFIX_LENGTH = 4;
static const size_t FILE_EXTENSION_LENGTH = 4;

/* Extern functions ---------------------------------------------------------*/
extern caddr_t heap_end;
extern char end asm("end");

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

  // TODO: if the board goes to error after reset, reinsert the card and reset again. The card has to loose the power
  // to go through the proper power-up sequence. Not sure if the device-reset-only sequence exists in the specs.

//// test the reset flags in order because the pin reset is always set.
//  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
//    resetReason = SOFT;
//  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
//    resetReason = POWER;
//  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
//    resetReason = PIN;
//  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
//    resetReason = LOWPOWER;
//  }
//
//// The flags must be cleared manually after use
//  __HAL_RCC_CLEAR_RESET_FLAGS();

  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  /* Configure User push-button in Interrupt mode */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Configure the system clock to 32 MHz */
  UART_Init();
  SystemClock_Config();
  SPI_Init();

  register caddr_t stack_ptr asm ("sp");
  int availableMemory = heap_end == NULL ? stack_ptr - &end : stack_ptr - heap_end;

  uart_print(snprintf(str, STR_LENGTH, "\r\nInit ready. Available memory = %d\r\n", availableMemory));
//  uart_print(snprintf(str, STR_LENGTH, "\r\nInit ready. Available memory = %d, Reset reason = %d\r\n", availableMemory, resetReason));

  if (FATFS_LinkDriver(&USER_Driver, USERPath) != 0) {
    Error_Handler("Init failed", 0);
  } else {
    uart_print(snprintf(str, STR_LENGTH, "FatFS link driver successful\r\n"));

    FRESULT res;
    if ((res = f_mount(&SDFatFs, USERPath, 0)) != FR_OK) {
      Error_Handler("Mount failed", res);
    } else {
      // Outer writing cycle
      DWORD freeSize = getFreeSize();
      while (freeSize > MAX_FILE_SIZE / 1024) { // while
        uart_print(snprintf(str, STR_LENGTH, "%ld kB available\r\n", freeSize));

        int newIndex = getLastFileIndex() + 1;
        if (newIndex < 1000) {

          char buffer[STR_LENGTH];
          snprintf(buffer, 64, "%s/%s%03d%s", DIRECTORY, FILE_PREFIX, newIndex, FILE_EXTENSION);
          uart_print(snprintf(str, STR_LENGTH, "Trying to create %s file\r\n", buffer));

          if ((res = f_open(&USERFile, buffer, FA_CREATE_ALWAYS|FA_WRITE)) != FR_OK) {
            Error_Handler("File creation failed", res);
          } else {
            // Let user know inner cycle started via board LEDs.
            BSP_LED_On(LED3);
            BSP_LED_On(LED4);
            HAL_Delay(200);
            BSP_LED_Off(LED3);
            BSP_LED_Off(LED4);
            HAL_Delay(200);

            // Inner writing cycle
            UINT bytesWritten = 0, blockWritten = 0;

            while (ubUserButtonClickEvent == RESET && bytesWritten < MAX_FILE_SIZE) {
              res = f_write(&USERFile, adcBuffer, ADC_BUFFER_LENGTH * sizeof(uint16_t), &blockWritten);
              bytesWritten += blockWritten;

              if (bytesWritten % 4000 == 0) {
                BSP_LED_Toggle(LED3);
              }
            }

            BSP_LED_Off(LED3);
            ubUserButtonClickEvent = RESET;

            if ((res = f_close(&USERFile)) != FR_OK) {
              Error_Handler("Close failed", res);
            }
          }
        } else {
          Error_Handler("Too many files created. Allowed amount < 1000", 0);
        }

        freeSize = getFreeSize();
      }
      uart_print(snprintf(str, STR_LENGTH, "Not enough space on SD card: %ld KB, required %d KB\r\n", freeSize,
                          MAX_FILE_SIZE / 1024));
      Error_Handler(NULL, 0);
    }

    FATFS_UnLinkDriver(USERPath);
  }
}

// Read example
//      if ((res = f_open(&USERFile, "test/test.mp4", FA_READ)) != FR_OK) {
//        Error_Handler("Open failed", res);
//      } else {
//        ReadLongFile(128); // buffers
//        printf("Done.\r\n");
//
//        if ((res = f_close(&USERFile)) != FR_OK) {
//          Error_Handler("Close failed", res);
//        }
//        BSP_LED_On(LED3);
//        for (;;) {}
//      }


static DIR dir;

static int getLastFileIndex() {
  FRESULT res = f_opendir(&dir, DIRECTORY);

  if (res != FR_OK) {
    Error_Handler("Can't open directory", res);
    return -1;
  } else {
    FILINFO fileInfo;

    int maxIndex = -1;
    char buffer[17];

    for (res = f_readdir(&dir, &fileInfo); res == FR_OK && fileInfo.fname[0]; res = f_readdir(&dir, &fileInfo)) {
      size_t sl = strlen(fileInfo.fname);

//      uart_print(snprintf(str, STR_LENGTH, "File = %s Size = %ld\r\n", fileInfo.fname, fileInfo.fsize));

      if (!(fileInfo.fattrib & AM_DIR) && sl >= 10 &&
          strncmp(FILE_PREFIX, fileInfo.fname, FILE_PREFIX_LENGTH) == 0 &&
          strncmp(FILE_EXTENSION, fileInfo.fname + sl - FILE_EXTENSION_LENGTH, FILE_EXTENSION_LENGTH) == 0) {

//        uart_print(snprintf(str, STR_LENGTH, "File = %s Size = %ld\r\n", fileInfo.lfname, fileInfo.fsize));

        size_t l = sl - (FILE_PREFIX_LENGTH + FILE_EXTENSION_LENGTH);
        if (l > 16) {
          l = 16;
        }

        strncpy(buffer, fileInfo.fname + FILE_PREFIX_LENGTH, l);
        buffer[l] = '\0';

        int i = (int) strtol(buffer, NULL, 10);

//        uart_print(snprintf(str, STR_LENGTH, "File buffer = %s, index = %d\r\n", buffer, i));

        if (i > maxIndex) {
          maxIndex = i;
        }
      }
    }

//    uart_print(snprintf(str, STR_LENGTH, "Max index = %d, res = %d\r\n", maxIndex, res));

    f_closedir(&dir);

    return maxIndex;
  }
}

/*
 * Calculate free size available on the mounted SD card KB.
 */
static DWORD getFreeSize()
{
  DWORD free_clust = 0;
  FATFS *fs = NULL;
  FRESULT res = f_getfree(DIRECTORY, &free_clust, &fs);
  if (res == FR_OK) {
    return free_clust * fs->csize / 2;
  } else {
    Error_Handler("f_getfree failed", res);
    return 0;
  }
}

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
//int __io_putchar(int ch)
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);
//
//  return ch;
//}

void uart_print(int strlen) {
  if (strlen > 1) {
    HAL_UART_Transmit(&UartHandle, (uint8_t *) &str, strlen, 0xFFFF);
  }
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
    uart_print(snprintf(str, STR_LENGTH, "%s: Res = %d\r\n", message, res));
  }

  /* Turn LED3 on */
  BSP_LED_On(LED4);
  for(;;) {}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  ubUserButtonClickEvent =  (GPIO_Pin == USER_BUTTON_PIN) ? SET : RESET;
}

//FRESULT ReadLongFile(uint16_t limit)
//{
//  uint16_t step=0;
//  UINT bytesRead;
//  uint32_t f_size = USERFile.fsize;
//
//  printf("File size: %lu\r\n", f_size);
//
//  for (uint32_t index = 0, count = 0; f_size > 0 && count < limit; index += step, count++) {
//    step = f_size < 512 ? f_size : 512;
//    f_size -= step;
//
//    f_lseek(&USERFile, index);
//    f_read (&USERFile, sect, step, &bytesRead);
//  }
//  return FR_OK;
//}
