/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2019, RExOS team
    All rights reserved.

    author: RJ (jam_roma@yahoo.com)
*/

#ifndef _NRF51_NRF_DRIVER_H_
#define _NRF51_NRF_DRIVER_H_

#include "sys_config.h"
#include "../object.h"
#include "../power.h"

//---------------------------------- ADC ---------------------------------------
typedef enum {
    NRF_ADC_AIN0 = 0,
    NRF_ADC_AIN1,
    NRF_ADC_AIN2,
    NRF_ADC_AIN3,
    NRF_ADC_AIN4,
    NRF_ADC_AIN5,
    NRF_ADC_AIN6,
    NRF_ADC_AIN7
} NRF_ADC_AIN;

typedef enum {
    NRF_ADC_INPUT_P0_26 = 0,
    NRF_ADC_INPUT_P0_27,
    NRF_ADC_INPUT_P0_01,
    NRF_ADC_INPUT_P0_02,
    NRF_ADC_INPUT_P0_03,
    NRF_ADC_INPUT_P0_04,
    NRF_ADC_INPUT_P0_05,
    NRF_ADC_INPUT_P0_06
} NRF_ADC_AINPUT;
//--------------------------------- POWER --------------------------------------
typedef enum {
    //if enabled
   NRF_POWER_GET_RESET_REASON = POWER_MAX
} NRF_POWER_IPCS;

typedef enum {
    RESET_REASON_UNKNOWN = 0,
    RESET_REASON_PIN_RST,
    RESET_REASON_WATCHDOG,
    RESET_REASON_SYSTEM_RESET_REQ,
    RESET_REASON_LOCKUP,
    RESET_REASON_WAKEUP,
    RESET_REASON_LPCOMP,
    RESET_REASON_DEBUG_INTERFACE_MODE,
} RESET_REASON;
//---------------------------------- GPIO --------------------------------------
typedef enum {
    // P0
    P0_0 = 0, P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, P0_9, P0_10,
    P0_11, P0_12, P0_13, P0_14, P0_15, P0_16, P0_17, P0_18, P0_19, P0_20,
    P0_21, P0_22, P0_23, P0_24, P0_25, P0_26, P0_27, P0_28, P0_29, P0_30, P0_31,
#if defined(NRF52840)
    // P1
    P1_0, P1_1, P1_2, P1_3, P1_4, P1_5, P1_6, P1_7, P1_8, P1_9, P1_10, P1_11,
    P1_12, P1_13, P1_14, P1_15, P1_16, P1_17, P1_18, P1_19, P1_20, P1_21,
    P1_22, P1_23, P1_24, P1_25, P1_26, P1_27, P1_28, P1_29, P1_30, P1_31,
#endif // NRF52840
    //only for service calls
    PIN_DEFAULT,
    PIN_UNUSED
} PIN;

typedef enum {
    PIN_MODE_INPUT,       ///< Input
    PIN_MODE_OUTPUT       ///< Output
} PIN_MODE;

typedef enum {
    PIN_PULL_NOPULL    = GPIO_PIN_CNF_PULL_Disabled,     ///<  Pin pullup resistor disabled
    PIN_PULL_DOWN      = GPIO_PIN_CNF_PULL_Pulldown,   ///<  Pin pulldown resistor enabled
    PIN_PULL_UP        = GPIO_PIN_CNF_PULL_Pullup,       ///<  Pin pullup resistor enabled
} PIN_PULL;

typedef enum {
    PIN_SENSE_NO        = GPIO_PIN_CNF_SENSE_Disabled,   ///<  Pin sense level disabled.
    PIN_SENSE_LOW       = GPIO_PIN_CNF_SENSE_Low,      ///<  Pin sense low level.
    PIN_SENSE_HIGH      = GPIO_PIN_CNF_SENSE_High,    ///<  Pin sense high level.
} PIN_SENSE;

#if defined(NRF52840)
#define GPIO_REG(pin)           ((NRF_GPIO_Type*)((pin > P0_31)? NRF_P1 : NRF_P0))
#else
#define GPIO_REG(pin)           (NRF_GPIO)
#endif //

#define PIN(pin)                ((pin > P0_31)? (pin - 32) : pin)

//---------------------------------- TIMER -------------------------------------
#define TIMER_CHANNEL_INVALID                        0xff

#define TIMER_MODE_CHANNEL_POS                       16
#define TIMER_MODE_CHANNEL_MASK                      (0xf << TIMER_MODE_CHANNEL_POS)

#if defined(NRF51) || defined(NRF52)
typedef enum {
    TIMER_0 = 0,
    TIMER_1,
    TIMER_2,
#if defined(NRF52)
    TIMER_3,
    TIMER_4,
#endif // NRF52
    TIMER_MAX
} TIMER_NUM;

typedef enum {
    TIMER_CC0 = 0,
    TIMER_CC1,
    TIMER_CC2,
    TIMER_CC3,
#if defined(NRF52)
    TIMER_CC4, /* TIMER3 TIMER4 only */
    TIMER_CC5, /* TIMER3 TIMER4 only */
#endif // NRF52
    TIMER_CC_MAX
} TIMER_CC;
#endif // NRF51 || NRF52

//------------------------------------ UART ------------------------------------
#if defined(NRF51) || defined(NRF52)
typedef enum {
    UART_0 = 0,
} UART_PORT;
#endif // NRF51

//------------------------------------ RTC -------------------------------------
#if defined(NRF51) || defined(NRF52)
typedef enum {
    RTC_0 = 0,
    RTC_1,
#if defined(NRF52)
    RTC_2,
#endif // NRF52
    RTC_MAX
} RTC_NUM;

typedef enum {
    RTC_CC0 = 0,
    RTC_CC1,
    RTC_CC2,
    RTC_CC3,
} RTC_CC;

#endif // NRF51 || NRF52

// ----------------------------------- RADIO -----------------------------------
#if defined(NRF51)
#define NRF_MAX_PACKET_LENGTH                   254
#endif // NRF51
#if defined(NRF52)
#define NRF_MAX_PACKET_LENGTH                   258
#endif // NRF52

typedef enum {
    RADIO_MODE_RF_1Mbit = 0,
    RADIO_MODE_RF_2Mbit,
#if defined(NRF52840)
    RADIO_MODE_RF_Ieee802154_250Kbit,
    RADIO_MODE_BLE_LR125Kbit,
    RADIO_MODE_BLE_LR500Kbit,
#endif
#if defined(NRF52832)
    RADIO_MODE_RF_250Kbit,
#endif
    RADIO_MODE_BLE_1Mbit,
#if defined(NRF52)
    RADIO_MODE_BLE_2Mbit
#endif //
} RADIO_MODE;

#endif /* _NRF51_NRF_DRIVER_H_ */
