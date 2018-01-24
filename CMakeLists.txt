cmake_minimum_required(VERSION 3.5)

set(TARGET_NAME "sdspi")
project(${TARGET_NAME} C ASM)

set(FIRMWARE_ROOT ../STM32Cube_FW_L1_V1.8.0)

include_directories(
        include
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Inc/
        ${FIRMWARE_ROOT}/Drivers/CMSIS/Include
        ${FIRMWARE_ROOT}/Drivers/CMSIS/Device/ST/STM32L1xx/Include
        ${FIRMWARE_ROOT}/Drivers/BSP/STM32L152C-Discovery
        ${FIRMWARE_ROOT}/Middlewares/Third_Party/FatFs/src
        ${FIRMWARE_ROOT}/Middlewares/Third_Party/FatFs/src/drivers
)

set(SOURCE_FILES
        src/main.c
        src/stm32l1xx_it.c
        src/system_stm32l1xx.c
        src/stm32l1xx_hal_msp.c
        src/syscalls.c
        src/sd.c
        src/user_diskio.c
        src/startup_stm32l152xb.s
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_cortex.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_gpio.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_uart.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_tim.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_tim_ex.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_adc.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_adc_ex.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_dma.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi.c
        ${FIRMWARE_ROOT}/Drivers/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi_ex.c
        ${FIRMWARE_ROOT}/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
        ${FIRMWARE_ROOT}/Middlewares/Third_Party/FatFs/src/ff.c
        ${FIRMWARE_ROOT}/Middlewares/Third_Party/FatFs/src/diskio.c
        ${FIRMWARE_ROOT}/Middlewares/Third_Party/FatFs/src/option/unicode.c
        ${FIRMWARE_ROOT}/Drivers/BSP/STM32L152C-Discovery/stm32l152c_discovery.c
)

set(CPU_PARM "-mcpu=cortex-m3 -mthumb -DSTM32L152xB")
set(LINKER_SCRIPT "../STM32L152RBTx_FLASH.ld")

set(CMAKE_C_FLAGS  "-fno-common -fno-exceptions -fdata-sections -ffunction-sections ${CPU_PARM} -Os -Wall")
set(CMAKE_EXE_LINKER_FLAGS  "-fno-exceptions ${CPU_PARM} -T ${LINKER_SCRIPT} -nostartfiles -Wl,-gc-sections")

add_executable("${TARGET_NAME}.elf" ${SOURCE_FILES})

add_custom_command(OUTPUT "${TARGET_NAME}.bin"
        POST_BUILD
        DEPENDS "${TARGET_NAME}.elf"
        COMMAND ${ARM_TOOLCHAIN}/bin/arm-none-eabi-objcopy -O binary "${TARGET_NAME}.elf" "${TARGET_NAME}.bin"
)

add_custom_target(bin DEPENDS "${TARGET_NAME}.bin")
