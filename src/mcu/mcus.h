#ifndef MCUS_H
#define MCUS_H

#include "mcudefs.h"

#if defined(MCU_STM32F4)
    #include "mcu_stm32f4.h"
#elif defined(MCU_STM32F7)
    #include "mcu_stm32f7.h"
#elif defined(MCU_RENESAS_RZT2M)
    #include "mcu_renesas_rzt2m.h"
#else
    #error "No MCU selected or MCU not supported."
#endif

#endif // MCUS_H
