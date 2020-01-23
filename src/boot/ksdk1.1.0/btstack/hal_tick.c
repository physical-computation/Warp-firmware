/*
 *  hal_tick.c
 *
 *  Based on implementation for MSP430 Experimenter board
 *
 */

// #include <msp430x54x.h>

#include <stdlib.h>

// #include "fsl_misc_utilities.h"
// #include "fsl_device_registers.h"
// #include "fsl_i2c_master_driver.h"
// #include "fsl_spi_master_driver.h"
// #include "fsl_rtc_driver.h"
// #include "fsl_clock_manager.h"
// #include "fsl_power_manager.h"
// #include "fsl_mcglite_hal.h"
// #include "fsl_port_hal.h"
// #include "fsl_hwtimer.h"

// #include "gpio_pins.h"
// #include "SEGGER_RTT.h"

// #include "warp.h"
#include "fsl_lptmr_driver.h"

#include "hal_tick.h"
#include "btstack_config.h"

// #define HWTIMER_LL_DEVIF kSystickDevif
// #define HWTIMER_LL_ID 0

// #define HWTIMER_ISR_PRIOR 5
// #define HWTIMER_PERIOD 100000
// #define HWTIMER_DOTS_PER_LINE 40
// #define HWTIMER_LINES_COUNT 2

// #include "hal_compat.h"

// extern const hwtimer_devif_t kSystickDevif;
// extern const hwtimer_devif_t kPitDevif;
// hwtimer_t hwtimer;

static void dummy_handler(void){};
static void (*tick_handler)(void) = &dummy_handler;

//use 32k rtc_clkout to get base ticks
//scale to get 4 counts per second = 8000 ticks

#define TIMER_COUNTDOWN 8000
#define LPTMR_INSTANCE 0

static lptmr_state_t gLPTMRState;
// uint8_t gLPTMR_counter;

// void hwtimer_callback(void *data)
// {
//     // SEGGER_RTT_printf(0,".");
//     // if ((HWTIMER_SYS_GetTicks(&hwtimer) % HWTIMER_DOTS_PER_LINE) == 0)
//     // {
//     //     SEGGER_RTT_printf(0,"\r\n");
//     // }
//     // if ((HWTIMER_SYS_GetTicks(&hwtimer) % (HWTIMER_LINES_COUNT * HWTIMER_DOTS_PER_LINE)) == 0)
//     // {
//     //     if (kHwtimerSuccess != HWTIMER_SYS_Stop(&hwtimer))
//     //     {
//     //         SEGGER_RTT_printf(0,"\r\nError: hwtimer stop.\r\n");
//     //     }
//     //     SEGGER_RTT_printf(0,"End\r\n");
//     // }
// }

void lptmr_isr_callback(void)
{
    // gLPTMR_counter++;
    (*tick_handler)();
    // printf("%d ",gLPTMR_counter);
}


void hal_tick_init(void){

    lptmr_user_config_t lptmrUserConfig =
    {
        .timerMode = kLptmrTimerModeTimeCounter, // Use LPTMR in Time Counter mode
        .freeRunningEnable = false, // When hit compare value, set counter back to zero
        .prescalerEnable = false, // bypass prescaler
        .prescalerClockSource = kClockLptmrSrcLpoClk, // use 1kHz Low Power Clock
        .isInterruptEnabled = true
    };

    // Initialize LPTMR
    LPTMR_DRV_Init(LPTMR_INSTANCE,&lptmrUserConfig,&gLPTMRState);

    // Set the timer period for 250 milliseconds
    LPTMR_DRV_SetTimerPeriodUs(LPTMR_INSTANCE,250000);

    // Specify the callback function when a LPTMR interrupt occurs
    LPTMR_DRV_InstallCallback(LPTMR_INSTANCE,lptmr_isr_callback);

    // Start counting
    LPTMR_DRV_Start(LPTMR_INSTANCE);

    SEGGER_RTT_printf(0, "\r\tStarted LPTMR\n");
    
    // // hardware_init();

    // // Print the initial banner
    // SEGGER_RTT_printf(0,"\r\nHwtimer Example \r\n");

    // // Hwtimer initialization
    // if (kHwtimerSuccess != HWTIMER_SYS_Init(&hwtimer, &HWTIMER_LL_DEVIF, HWTIMER_LL_ID, NULL))
    // {
    //     SEGGER_RTT_printf(0,"\r\nError: hwtimer initialization.\r\n");
    // }

    // /*
    //  * In case you wish to raise the Systick ISR priority, then use the below command with the
    //  * appropriate priority setting:
    //  * NVIC_SetPriority(SysTick_IRQn, isrPrior);
    //  */
    // NVIC_SetPriority(SysTick_IRQn, HWTIMER_ISR_PRIOR);

    // if (kHwtimerSuccess != HWTIMER_SYS_SetPeriod(&hwtimer, HWTIMER_PERIOD))
    // {
    //     SEGGER_RTT_printf(0,"\r\nError: hwtimer set period.\r\n");
    // }
    // if (kHwtimerSuccess != HWTIMER_SYS_RegisterCallback(&hwtimer, hwtimer_callback, NULL))
    // {
    //     SEGGER_RTT_printf(0,"\r\nError: hwtimer callback registration.\r\n");
    // }
    // if (kHwtimerSuccess != HWTIMER_SYS_Start(&hwtimer))
    // {
    //     SEGGER_RTT_printf(0,"\r\nError: hwtimer start.\r\n");
    // }

    // // TA1CCTL0 = CCIE;                  // CCR0 interrupt enabled
    // // TA1CTL = TASSEL_1 | MC_2 | TACLR; // use ACLK (32768), contmode, clear TAR
    // // TA1CCR0 = TIMER_COUNTDOWN;        // -> 1/4 s
}

void hal_tick_set_handler(void (*handler)(void)) { //this will get called every "hal_tick_get_tick_period_in_ms"

    if (handler == NULL)
    {
        tick_handler = &dummy_handler;
        return;
    }
    tick_handler = handler;
}

int hal_tick_get_tick_period_in_ms(void){
    return 250;
}
