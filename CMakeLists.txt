#此文件从模板自动生成! 请勿更改!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.28.1)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(COD_H7_Template C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(
        -mcpu=cortex-m7
        -mthumb
        -mthumb-interwork
        -mfpu=fpv5-d16      # 双精度浮点单元
        -mfloat-abi=hard    # 硬件浮点
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fmessage-length=0
)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(
        Core/Inc
        USB_DEVICE/App
        USB_DEVICE/Target
        Drivers/STM32H7xx_HAL_Driver/Inc
        Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        Drivers/CMSIS/Device/ST/STM32H7xx/Include
        Drivers/CMSIS/Include
        Middlewares/ST/ARM/DSP/Inc
        SystemView/SEGGER
        SystemView/Config
        SystemView/Sample/FreeRTOSV10
        Application/Task/Inc
        BSP/Inc
        Components/Algorithm/Inc
        Components/Controller/Inc
        Components/Device/Inc
)



add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx)

#file(GLOB_RECURSE SOURCES "Core/*.*" "Middlewares/*.*" "USB_DEVICE/*.*" "Drivers/*.*"
#        "Application/Task/Src/*.*" "BSP/Src/*.*"
#        "Components/Algorithm/Src/*.*" "Components/Controller/Src/*.*" "Components/Device/Src/*.*"
#        "SystemView/*.*")
set(sources
    USB_DEVICE/Target/usbd_conf.c
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usbd_cdc_if.c
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/freertos.c
    Core/Src/adc.c
    Core/Src/bdma.c
    Core/Src/dma.c
    Core/Src/fdcan.c
    Core/Src/spi.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    Core/Src/system_stm32h7xx.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    startup_stm32h723xx.s
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H723VGTX_FLASH.ld)

#add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
# 修改为：
add_link_options(
        -mcpu=cortex-m7
        -mthumb
        -mthumb-interwork
        -mfpu=fpv5-d16      # 双精度浮点单元
        -mfloat-abi=hard    # 硬件浮点
        -Wl,-gc-sections
        -Wl,--print-memory-usage
        -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
        -T ${LINKER_SCRIPT}
)
add_link_options(-T ${LINKER_SCRIPT})

link_directories(Middlewares/ST/ARM/DSP/Lib)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

add_subdirectory(cmake/stm32cubemx)

# 在主 CMakeLists.txt 中添加浮点选项
add_compile_options(
        -mcpu=cortex-m7
        -mthumb
        -mthumb-interwork
        # 添加浮点选项（对于 STM32H7）
        -mfpu=fpv5-d16
        -mfloat-abi=hard
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fmessage-length=0
)

# 同样在链接选项中添加
add_link_options(
        -mcpu=cortex-m7
        -mthumb
        -mthumb-interwork
        # 添加浮点选项
        -mfpu=fpv5-d16
        -mfloat-abi=hard
        -Wl,-gc-sections
        --print-memory-usage
        -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
)

# 链接子目录中创建的库
target_link_libraries(${PROJECT_NAME}.elf PUBLIC
        STM32_Drivers
        USB_Device_Library
        FreeRTOS
        Component
        BSP
        SystemView
        Task
        -larm_cortexM7lfdp_math
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

#add_subdirectory(cmake/stm32cubemx)