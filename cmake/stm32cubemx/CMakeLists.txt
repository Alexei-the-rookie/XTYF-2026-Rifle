cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32H723xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
	${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/ARM/DSP/Inc
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Algorithms/Inc
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Controller/Inc
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Device/Inc
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/Target/usbd_conf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usbd_cdc_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/freertos.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/bdma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/fdcan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32h7xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32h7xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32h7xx_hal_timebase_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32h723xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32h7xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
)

# Drivers Middlewares


set(USB_Device_Library_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)
set(FreeRTOS_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/port.c
)

# Link directories setup
set(BSP_Src
	${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/Src/bsp_adc.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/Src/bsp_can.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/Src/bsp_dwt.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/Src/bsp_gpio.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/Src/bsp_mcu.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/Src/bsp_spi.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/Src/bsp_pwm.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/Src/bsp_tick.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../BSP/Src/bsp_uart.c
)

set(SystemView_Src
	${CMAKE_CURRENT_SOURCE_DIR}/../../SystemView/SEGGER/SEGGER_SYSVIEW.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../SystemView/SEGGER/SEGGER_RTT.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../SystemView/SEGGER/SEGGER_RTT_printf.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../SystemView/Config/SEGGER_SYSVIEW_Conf.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../SystemView/Sample/FreeRTOSV10/SEGGER_SYSVIEW_FreeRTOS.c
)

set(Component_src
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Algorithm/Src/CRC.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Algorithm/Src/Kalman_Filter.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Algorithm/Src/LPF.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Algorithm/Src/Quaternion.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Algorithm/Src/Ramp.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Algorithm/Src/RLS.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Controller/Src/PID.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Device/Src/Bmi088.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Device/Src/Image_Transmission.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Device/Src/MiniPC.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Device/Src/Motor.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Device/Src/Referee_System.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Components/Device/Src/Remote_control.c
)

set(Task_Src
	${CMAKE_CURRENT_SOURCE_DIR}/../../Application/Task/Src/CAN_Task.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Application/Task/Src/Control_Task.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Application/Task/Src/INS_Task.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../Application/Task/Src/Detect_Task.c
)

# 在文件顶部或 MX_LINK_LIBS 定义之前添加
#link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/ARM/DSP/Lib)

# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    USB_Device_Library
	FreeRTOS
	Component
	BSP
	SystemView
	Task
	-larm_cortexM7lfdp_math
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

# Create Component static library
add_library(Component OBJECT
        ../../Components/Controller/Src/LQR.c
		../../Components/Device/Src/AK_motor.c)
target_sources(Component PRIVATE ${Component_src})
target_link_libraries(Component PUBLIC stm32cubemx)

add_library(BSP OBJECT)
target_sources(BSP PRIVATE ${BSP_Src})
target_link_libraries(BSP PUBLIC stm32cubemx)

add_library(SystemView OBJECT)
target_sources(SystemView PRIVATE ${SystemView_Src})
target_link_libraries(SystemView PUBLIC stm32cubemx)

# Create USB_Device_Library static library
add_library(USB_Device_Library OBJECT)
target_sources(USB_Device_Library PRIVATE ${USB_Device_Library_Src})
target_link_libraries(USB_Device_Library PUBLIC stm32cubemx)

# Create FreeRTOS static library
add_library(FreeRTOS OBJECT)
target_sources(FreeRTOS PRIVATE ${FreeRTOS_Src})
target_link_libraries(FreeRTOS PUBLIC stm32cubemx)

add_library(Task OBJECT)
target_sources(Task PRIVATE ${Task_Src})
target_link_libraries(Task PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME}.elf PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME}.elf PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME}.elf PRIVATE ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME}.elf PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
