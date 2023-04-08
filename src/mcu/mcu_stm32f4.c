#include "mcu.h"

void mcu_gpio_init(void) {

    // Enable the clock for GPIO Port A
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Set the GPIO pins for motor control as outputs
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure the alternate function (AF) for the motor control pins
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_TIM2);

}


void mcu_adc_init(void) {

    // Enable the clock for the ADCs
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // Initialize the ADCs for motor current and bus voltage measurements
    ADC_InitTypeDef ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;

    // Configure the ADC common settings
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);

    // Configure the ADC settings for motor current measurements
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStruct);

    // Configure the ADC channel for motor current measurements (PA0)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles);

    // Configure the ADC settings for bus voltage measurements
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStruct);

    // Configure the ADC channel for bus voltage measurements (PA1)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_28Cycles);

    // Enable the ADC1
    ADC_Cmd(ADC1, ENABLE);

    // Wait for the ADC1 to be ready
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS)) {}

    // Configure the ADC1 channels for the motor current and bus voltage measurements
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_28Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_28Cycles);
}


void mcu_uart_init(void) {
    // Initialize UART 2 with a baud rate of 115200
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStruct);
    USART_Cmd(USART2, ENABLE);
}


void mcu_timer_init(uint8_t timer_num, uint32_t freq_hz) {
    // Enable the timer clock
    if (timer_num == 1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    } else if (timer_num == 2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    } else if (timer_num == 3) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    } else if (timer_num == 4) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    } else {
        // Unsupported timer number
        return;
    }

    // Initialize the timer
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_InitStruct.TIM_Period = (SystemCoreClock / freq_hz) - 1;
    TIM_InitStruct.TIM_Prescaler = 0;
    TIM_InitStruct.TIM_ClockDivision = 0;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &TIM_InitStruct);

    // Enable the timer
    TIM_Cmd(TIMx, ENABLE);
}



void mcu_pwm_init(uint8_t timer_num, uint8_t channel_num, uint32_t freq_hz) {
    // Enable the clock for the timer
    if (timer_num == 1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    } else if (timer_num == 2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    } else if (timer_num == 3) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    } else if (timer_num == 4) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    } else {
        // Invalid timer number, do nothing
        return;
    }

    // Configure the timer
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / freq_hz) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    // Configure the PWM output channel
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    if (channel_num == 1) {
        TIM_OC1Init(TIMx, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
    } else if (channel_num == 2) {
        TIM_OC2Init(TIMx, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
    } else if (channel_num == 3) {
        TIM_OC3Init(TIMx, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
    } else if (channel_num == 4) {
        TIM_OC4Init(TIMx, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
    } else {
        // Invalid channel number, do nothing
        return;
    }

    // Start the timer
    TIM_Cmd(TIMx, ENABLE);
}


void mcu_adc_start_conversion(uint8_t channel_num) {
    // Start the ADC conversion for the specified channel
    ADC_RegularChannelConfig(ADC1, channel_num, 1, ADC_SampleTime_3Cycles);
    ADC_SoftwareStartConv(ADC1);
}


uint16_t mcu_adc_read(uint8_t channel_num) {
    // Read and return the value of the ADC conversion for the specified channel
    ADC_RegularChannelConfig(ADC1, channel_num, 1, ADC_SampleTime_3Cycles);
    ADC_SoftwareStartConv(ADC1);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}


void mcu_pwm_set_duty_cycle(uint8_t timer_num, uint8_t channel_num, float duty_cycle) {
    // Calculate the pulse width value based on the desired duty cycle
    uint16_t pulse_width = (uint16_t) (duty_cycle * (TIM_ARR + 1));

    // Set the pulse width for the specified channel on the specified timer
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = pulse_width;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    switch (channel_num) {
        case 1:
            TIM_OC1Init(TIM2, &TIM_OCInitStruct);
            break;
        case 2:
            TIM_OC2Init(TIM2, &TIM_OCInitStruct);
            break;
        case 3:
            TIM_OC3Init(TIM2, &TIM_OCInitStruct);
            break;
        case 4:
            TIM_OC4Init(TIM2, &TIM_OCInitStruct);
            break;
        default:
            break;
    }

    // Start the timer
    TIM_Cmd(TIM2, ENABLE);
}


uint32_t mcu_timer_get_counter_value(uint8_t timer_num) {
    // Get the current counter value of the specified timer
    uint32_t counter_value = 0;
    switch (timer_num) {
        case 1:
            counter_value = TIM_GetCounter(TIM2);
            break;
        case 2:
            counter_value = TIM_GetCounter(TIM3);
            break;
        case 3:
            counter_value = TIM_GetCounter(TIM4);
            break;
        case 4:
            counter_value = TIM_GetCounter(TIM5);
            break;
        default:
            break;
    }
    return counter_value;
}

