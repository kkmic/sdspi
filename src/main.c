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
#include "uart_print.c"

/* Timer frequency (unit: Hz). With SysClk set to 32MHz, timer frequency TIMER_FREQUENCY_HZ range is min=1Hz, max=32.719kHz. */
#define TIMER_FREQUENCY_HZ          1000

/* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */
#define ADC_CHANNELS 4

/* Global structures and functions */
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef TimHandle;
ADC_HandleTypeDef AdcHandle;

/* Variable to report ADC sequencer status */
__IO uint8_t ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */

/* Variables to manage push button on board: interface between ExtLine interruption and main program */
__IO uint8_t ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */

/* Private structures ---------------------------------------------------------*/
static DIR dir;
static FIL USERFile;
static FATFS SDFatFs;
static char USERPath[4];   /* USER logical drive path */

void Error_Handler(const char* message, uint8_t res);

/* Private functions ---------------------------------------------------------*/
static void SystemClock_Config(void);
static void SPI_Init(void);
static void TIM_Init(void);
static void ADC_Init(void);

#define ADC_BUFFER_LENGTH 200
static __IO uint16_t adcBuffer[ADC_BUFFER_LENGTH];

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

  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  SystemClock_Config();

  if (uart_init() != HAL_OK) {
    Error_Handler(NULL, 0);
  }

  register caddr_t stack_ptr asm ("sp");
  int availableMemory = heap_end == NULL ? stack_ptr - &end : stack_ptr - heap_end;

  uart_print(snprintf(buffer, STRING_LENGTH, "\r\nInit ready. Available memory = %d\r\n", availableMemory));
//  uart_print(snprintf(buffer, STRING_LENGTH, "\r\nInit ready. Available memory = %d, Power On = %s\r\n", availableMemory, powerOn == ENABLE ? "True" : "False"));

  /* Configure User push-button in Interrupt mode */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  SPI_Init();
  ADC_Init();
  TIM_Init();

  if (FATFS_LinkDriver(&USER_Driver, USERPath) != 0) {
    Error_Handler("Init failed", 0);
  } else {
    uart_print(snprintf(buffer, STRING_LENGTH, "FatFS link driver successful\r\n"));

    FRESULT ret;
    if ((ret = f_mount(&SDFatFs, USERPath, 0)) != FR_OK) {
      Error_Handler("Mount failed", ret);
    } else {
      char filename[STRING_LENGTH];

      // Outer writing cycle. Create file until there is space on the disk.
      DWORD freeSize = 0;
      for (freeSize = getFreeSize(); freeSize > MAX_FILE_SIZE / 1024; freeSize = getFreeSize()) {
        uart_print(snprintf(buffer, STRING_LENGTH, "%ld kB available\r\n", freeSize));

        int newIndex = getLastFileIndex() + 1;

        if (newIndex < 1000) { // Protect against long file names.
          snprintf(filename, 64, "%s/%s%03d%s", DIRECTORY, FILE_PREFIX, newIndex, FILE_EXTENSION);
          uart_print(snprintf(buffer, STRING_LENGTH, "Creating %s\r\n", filename));

          if ((ret = f_open(&USERFile, filename, FA_CREATE_ALWAYS|FA_WRITE)) != FR_OK) {
            Error_Handler("File creation failed", ret);
          } else {
            if (HAL_TIM_Base_Start(&TimHandle) != HAL_OK ||
                HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adcBuffer, ADC_BUFFER_LENGTH) != HAL_OK) {
              Error_Handler("Can't start Timer/DMA", 0);
            }

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

              // Wait until DMA completes all buffer ADC sampling
              while (ubUserButtonClickEvent == RESET && ubSequenceCompleted == RESET);

              if (ubSequenceCompleted == SET) {
                ret = f_write(&USERFile, (void*)adcBuffer, ADC_BUFFER_LENGTH * sizeof(uint16_t), &blockWritten);
                bytesWritten += blockWritten;

                ubSequenceCompleted = RESET;

                if (bytesWritten % TIMER_FREQUENCY_HZ == 0) {
//                  uart_print(snprintf(buffer, STRING_LENGTH, "%d bytes written\r", bytesWritten));
                  BSP_LED_Toggle(LED3);
                }
              }
            }

            uart_print(snprintf(buffer, STRING_LENGTH, "\r\nFlushing data to file, %d bytes written\r\n", bytesWritten));

            BSP_LED_Off(LED3);
            ubUserButtonClickEvent = RESET;

            if (HAL_ADC_Stop_DMA(&AdcHandle) != HAL_OK || HAL_TIM_Base_Stop(&TimHandle) != HAL_OK) {
              Error_Handler("Can't stop Timer/DMA", 0);
            }
            ubSequenceCompleted = RESET;

            if ((ret = f_close(&USERFile)) != FR_OK) {
              Error_Handler("Close failed", ret);
            }
          }
        } else {
          Error_Handler("Too many files created. Allowed amount < 1000", 0);
        }
      }
      uart_print(snprintf(buffer, STRING_LENGTH, "Not enough space on SD card: %ld KB, required %d KB\r\n", freeSize,
                          MAX_FILE_SIZE / 1024));
      Error_Handler(NULL, 0);
    }

    FATFS_UnLinkDriver(USERPath);
  }
}

static int getLastFileIndex() {
  FRESULT ret = f_opendir(&dir, DIRECTORY);

  if (ret != FR_OK) {
    Error_Handler("Can't open directory", ret);
    return -1;
  } else {
    FILINFO fileInfo;

    int maxIndex = -1;
    char buffer[17];

    for (ret = f_readdir(&dir, &fileInfo); ret == FR_OK && fileInfo.fname[0]; ret = f_readdir(&dir, &fileInfo)) {
      size_t sl = strlen(fileInfo.fname);

//      uart_print(snprintf(buffer, STRING_LENGTH, "File = %s Size = %ld\r\n", fileInfo.fname, fileInfo.fsize));

      if (!(fileInfo.fattrib & AM_DIR) && sl >= 10 &&
          strncmp(FILE_PREFIX, fileInfo.fname, FILE_PREFIX_LENGTH) == 0 &&
          strncmp(FILE_EXTENSION, fileInfo.fname + sl - FILE_EXTENSION_LENGTH, FILE_EXTENSION_LENGTH) == 0) {

//        uart_print(snprintf(buffer, STRING_LENGTH, "File = %s Size = %ld\r\n", fileInfo.lfname, fileInfo.fsize));

        size_t l = sl - (FILE_PREFIX_LENGTH + FILE_EXTENSION_LENGTH);
        if (l > 16) {
          l = 16;
        }

        strncpy(buffer, fileInfo.fname + FILE_PREFIX_LENGTH, l);
        buffer[l] = '\0';

        int i = (int) strtol(buffer, NULL, 10);

//        uart_print(snprintf(buffer, STRING_LENGTH, "File buffer = %s, index = %d\r\n", buffer, i));

        if (i > maxIndex) {
          maxIndex = i;
        }
      }
    }

//    uart_print(snprintf(buffer, STRING_LENGTH, "Max index = %d, ret = %d\r\n", maxIndex, ret));

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
  FRESULT ret = f_getfree(DIRECTORY, &free_clust, &fs);
  if (ret == FR_OK) {
    return free_clust * fs->csize / 2;
  } else {
    Error_Handler("f_getfree failed", ret);
    return 0;
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

  HAL_StatusTypeDef ret = HAL_SPI_Init(&hspi2);
  if (ret != HAL_OK) {
    Error_Handler("SPI module init error", ret);
  }
}

static void TIM_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* Time Base configuration */
  TimHandle.Instance = TIMx;

  /* Configure timer frequency */
  /* Note: Setting of timer prescaler to 489 to increase the maximum range    */
  /*       of the timer, to fit within timer range of 0xFFFF.                 */
  /*       Setting of reload period to SysClk/489 to maintain a base          */
  /*       frequency of 1us.                                                  */
  /*       With SysClk set to 32MHz, timer frequency (defined by label        */
  /*       TIMER_FREQUENCY_HZ range) is min=1Hz, max=32.719kHz.               */
  /* Note: Timer clock source frequency is retrieved with function            */
  /*       HAL_RCC_GetPCLK1Freq().                                            */
  /*       Alternate possibility, depending on prescaler settings:            */
  /*       use variable "SystemCoreClock" holding HCLK frequency, updated by  */
  /*       function HAL_RCC_ClockConfig().                                    */
  TimHandle.Init.Period = ((HAL_RCC_GetPCLK1Freq() / (489 * TIMER_FREQUENCY_HZ)) - 1);
  TimHandle.Init.Prescaler = (489 - 1);
  TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

  HAL_StatusTypeDef ret = HAL_TIM_Base_Init(&TimHandle);
  if (ret != HAL_OK) {
    Error_Handler("Timer initialization error", ret);
  }

  /* Timer TRGO selection */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  ret = HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);
  if (ret != HAL_OK) {
    Error_Handler("Timer TRGO selection error", ret);
  }
}

/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
static void ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  /* Configuration of AdcHandle init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADCx;

//  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK) {
//    /* ADC initialization error */
//    Error_Handler(4);
//  }

  AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;               /* Sequencer enabled (ADC conversion on several channels, successively, following settings below) */
  AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  AdcHandle.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  AdcHandle.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  AdcHandle.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  AdcHandle.Init.ContinuousConvMode = DISABLE;                     /* Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled */
  AdcHandle.Init.NbrOfConversion = ADC_CHANNELS; /* Sequencer of regular group will convert the 4 first ranks: rank1, rank2, rank3, rank4 */
  AdcHandle.Init.DiscontinuousConvMode = ENABLE;                   /* Sequencer of regular group will convert the sequence in several sub-divided sequences */
  AdcHandle.Init.NbrOfDiscConversion = ADC_CHANNELS;                          /* Sequencer of regular group will convert ranks one by one, at each conversion trig */
  AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_Tx_TRGO;  /* Trig of conversion start done by external event */
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  AdcHandle.Init.DMAContinuousRequests = ENABLE;

  HAL_StatusTypeDef ret = HAL_ADC_Init(&AdcHandle);
  if (ret != HAL_OK) {
    Error_Handler("ADC initialization error", ret);
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each ADC conversion (IT by DMA end  */
  /*       of transfer), select sampling time and ADC clock with sufficient   */
  /*       duration to not create an overhead situation in IRQHandler.        */
  /* Note: Set long sampling time due to internal channels (VrefInt,          */
  /*       temperature sensor) constraints.                                   */
  /*       For example, sampling time of temperature sensor must be higher    */
  /*       than 4us. Refer to device datasheet for min/typ/max values.        */
  sConfig.Channel = ADCx_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_96CYCLES;

  ret = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
  if (ret != HAL_OK) {
    Error_Handler("Channel 1 Configuration Error", ret);
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 2 */
  /* Replicate previous rank settings, change only channel and rank */
  sConfig.Channel = ADCx_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;

  ret = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
  if (ret != HAL_OK) {
    Error_Handler("Channel 2 Configuration Error", ret);
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 3 */
  /* Replicate previous rank settings, change only channel and rank */
  sConfig.Channel = ADCx_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;

  ret = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
  if (ret != HAL_OK) {
    Error_Handler("Channel 3 Configuration Error", ret);
  }

  sConfig.Channel = ADCx_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;

  ret = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
  if (ret != HAL_OK) {
    Error_Handler("Channel 4 Configuration Error", ret);
  }
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  /* Report to main program that ADC sequencer has reached its end */
  ubSequenceCompleted = SET;
}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  Error_Handler("ADC error", hadc->ErrorCode);
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
void Error_Handler(const char* message, uint8_t res)
{
  if (message != NULL) {
    uart_print(res > 0 ?
               snprintf(buffer, STRING_LENGTH, "%s: Res = %d\r\n", message, res) :
               snprintf(buffer, STRING_LENGTH, "%s\r\n", message));
  }

  /* Turn LED3 on */
  BSP_LED_On(LED4);
  for(;;) {}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  ubUserButtonClickEvent = (GPIO_Pin == USER_BUTTON_PIN) ? SET : RESET;
//
//  uart_print(snprintf(buffer, STRING_LENGTH, "\r\nUser button clicked = %s\r\n", ubUserButtonClickEvent == SET ? "True" : "False"));
}
