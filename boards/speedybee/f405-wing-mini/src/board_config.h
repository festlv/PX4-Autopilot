/****************************************************************************
 *
 *   Copyright (c) 2018, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * board internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* speedybeef405 GPIOs ***********************************************************************************/
/* LEDs */
// power - red
// LED1 - PA13 - green
// LED2 - PA14 - blue
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN13)
#define GPIO_LED_BLUE   GPIO_LED1

#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN14)
#define GPIO_LED_GREEN   GPIO_LED2


#define BOARD_OVERLOAD_LED     LED_BLUE

#define  FLASH_BASED_PARAMS

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */

#define ADC_BATTERY_VOLTAGE_CHANNEL  10
#define ADC_BATTERY_CURRENT_CHANNEL  11
#define ADC_RC_RSSI_CHANNEL          14
#define ADC_AIRSPEED_CHANNEL         15

#define ADC_CHANNELS (1 << ADC_RC_RSSI_CHANNEL) | (1 << ADC_BATTERY_CURRENT_CHANNEL) | (1 << ADC_BATTERY_VOLTAGE_CHANNEL) | ( 1 << ADC_AIRSPEED_CHANNEL)

/* User GPIOs
 *
 * GPIO0-5 are the PWM servo outputs.
 * GPIO_TIM3_CH3OUT        GPIO_TIM3_CH3OUT_1 //PB0 S1_OUT D1_ST7
 * GPIO_TIM3_CH4OUT        GPIO_TIM3_CH4OUT_1 //PB1 S2_OUT D1_ST2
 * GPIO_TIM2_CH4OUT        GPIO_TIM2_CH4OUT_1 //PA3 S3_OUT D1_ST6
 * GPIO_TIM2_CH3OUT        GPIO_TIM2_CH3OUT_1 //PA2 S4_OUT D1_ST1
 * GPIO_TIM5_CH2OUT        GPIO_TIM5_CH2OUT_1 //PA1 S5_OUT
 * GPIO_TIM1_CH1OUT        GPIO_TIM1_CH1OUT_1 //PA8 S6_OUT
 */

#define _MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))

#define GPIO_GPIO0_INPUT             _MK_GPIO_INPUT(GPIO_TIM4_CH1IN)
#define GPIO_GPIO1_INPUT             _MK_GPIO_INPUT(GPIO_TIM4_CH2IN)
#define GPIO_GPIO2_INPUT             _MK_GPIO_INPUT(GPIO_TIM4_CH3IN)
#define GPIO_GPIO3_INPUT             _MK_GPIO_INPUT(GPIO_TIM4_CH4IN)
//#define GPIO_GPIO4_INPUT             _MK_GPIO_INPUT(GPIO_TIM5_CH2IN)
//#define GPIO_GPIO5_INPUT             _MK_GPIO_INPUT(GPIO_TIM1_CH1IN)

#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))

#define GPIO_GPIO0_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM4_CH1OUT)
#define GPIO_GPIO1_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM4_CH2OUT)
#define GPIO_GPIO2_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM4_CH3OUT)
#define GPIO_GPIO3_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM4_CH4OUT)
//#define GPIO_GPIO4_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM5_CH2OUT)
//#define GPIO_GPIO5_OUTPUT            _MK_GPIO_OUTPUT(GPIO_TIM1_CH1OUT)

/* USB OTG FS
 * This board does not have VBUS sensing, see src/usb.c for a stubbed USB connected detection
 */

/* PWM
 *
 */
#define DIRECT_PWM_OUTPUT_CHANNELS      9
#define BOARD_NUM_IO_TIMERS 4

/* High-resolution timer */
#define HRT_TIMER                    1 // T1C1
#define HRT_TIMER_CHANNEL            1 // use capture/compare channel 1

//#define HRT_PPM_CHANNEL              3 // capture/compare channel 3
//#define GPIO_PPM_IN                  (GPIO_ALT|GPIO_AF2|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN8)

#define RC_SERIAL_PORT               "/dev/ttyS0"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* Tone alarm output */
#define GPIO_TONE_ALARM_IDLE    /* PC15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)
#define GPIO_TONE_ALARM_GPIO    /* PC15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)

/*
 * One RC_IN
 *
 * GPIO PPM_IN on PB8 T4CH3
 * SPEKTRUM_RX (it's TX or RX in Bind) on PA10 UART1
 * The FMU can drive GPIO PPM_IN as an output
 */
// TODO?
//#define GPIO_PPM_IN_AS_OUT            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
//#define SPEKTRUM_RX_AS_GPIO_OUTPUT()  px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
//#define SPEKTRUM_RX_AS_UART()         px4_arch_configgpio(GPIO_USART1_RX)
//#define SPEKTRUM_OUT(_one_true)       px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define BOARD_HAS_ON_RESET 1

#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE (1024*3)


__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);


/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
