# Include the appropriate implementation file based on the selected MCU
if(MCU STREQUAL "STM32F4")
    target_sources(mcu PRIVATE mcu_stm32f4.c)
elseif(MCU STREQUAL "STM32F7")
    target_sources(mcu PRIVATE mcu_stm32f7.c)
elseif(MCU STREQUAL "RZT2M")
    target_sources(mcu PRIVATE mcu_rzt2m.c)
elseif(MCU STREQUAL "RZA")
    target_sources(mcu PRIVATE mcu_rza.c)
else()
    message(FATAL_ERROR "Unsupported MCU")
endif()