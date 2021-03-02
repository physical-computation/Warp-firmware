#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_os_abstraction.h"
#include "fsl_interrupt_manager.h"
#include "fsl_power_manager.h"
#include "fsl_gpio_driver.h"
#include "fsl_llwu_hal.h"
#include "fsl_smc_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_sim_hal.h"
#include "fsl_misc_utilities.h"
#include "fsl_rtc_driver.h"
#include "fsl_spi_master_driver.h"

#include "config.h"
#include "glaux.h"
#include "warp.h"
#include "gpio_pins.h"
#include "devRV8803C7.h"



/*
 *	From KSDK power_manager_demo.c BEGIN>>>
 */
#if (!WARP_BUILD_ENABLE_DEVRV8803C7)
	static void
	setSleepRtcAlarm(uint32_t offsetSec)
	{
		rtc_datetime_t date;
		uint32_t seconds;

		// get date time and convert to seconds
		RTC_DRV_GetDatetime(0, &date);

		// convert to sec and add offset
		RTC_HAL_ConvertDatetimeToSecs(&date, &seconds);

		//TODO: should check for overflow...
		seconds += offsetSec;
		RTC_HAL_ConvertSecsToDatetime(&seconds, &date);

		// set the datetime for alarm
		if (RTC_DRV_SetAlarm(0, &date, true))
		{
			//...
		}
		else
		{
			return;
		}
	}
#endif

void
gpioDisableWakeUp(void)
{
	/*
	 *	Disables interrupt on LLWU_Px. The BOARD_* defines are defined in warp.h
	 */
	PORT_HAL_SetPinIntMode(BOARD_SW_LLWU_BASE, BOARD_SW_LLWU_PIN, kPortIntDisabled);
	INT_SYS_DisableIRQ(BOARD_SW_LLWU_IRQ_NUM);
}



void
gpioEnableWakeUp(void)
{
	/*
	 *	To make assurance doubly sure, turn off RTC based on Errata 8068 / AN4503.
	 *	Revisit the need for this once we have more measurements:
	 *
	 *	For KL03 chip errata 8068. See https://community.nxp.com/thread/350259,
	 *	combined with ideas from AN4503 page 2.
	 *
	 *	Enabling this block will kill the RTC, which means no sleep routines, etc.
	 */
	volatile unsigned int	dummyread;

	__asm("CPSID i");			/*	Disable interrupts		*/
	SIM->COPC=0x00;				/*	Disable COP watchdog		*/
	dummyread = SIM->COPC;			/*	Read-after-write sequence	*/

	/*
	 *	Errata 8068 fix
	 */
	SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;	/*	Enable clock to RTC				*/
	dummyread = SIM->SCGC6;			/*	Read-after-write sequence			*/
	RTC->TSR = 0x00;			/*	Dummy write to RTC TSR per errata 8068		*/
	dummyread = RTC->TSR;			/*	Read-after-write sequence			*/
	SIM->SCGC6 &= ~SIM_SCGC6_RTC_MASK;	/*	Disable clock to RTC				*/
	dummyread = SIM->SCGC6;			/*	Read-after-write sequence			*/



	/*
	 *	See also AN4503, Section 3.1.6.
	 *
	 *	First, enable the PORTA clock but disable the PORTB clock.
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_DisablePortClock(1);



	/*
	 *
	 *	Next, make PTA0 switch from being SWD to being PTA0/LLWU_P7/IRQ0
	 *	We don't need to revert this later, since waking up from VLLx is done
	 *	through a soft reset and that config is lost? (TODO: double check.)
	 */
	GPIO_DRV_Init(wakeupPins  /* input pins */, NULL  /* output pins */);
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);

	/*
	 *	Enables any edge interrupt for the LLWU_Px pin
	 */
	PORT_HAL_SetPinIntMode(BOARD_SW_LLWU_BASE, BOARD_SW_LLWU_PIN, kPortIntEitherEdge);
	INT_SYS_EnableIRQ(BOARD_SW_LLWU_IRQ_NUM);

	/*
	 *	Redundant? Check. 
	 */
	INT_SYS_EnableIRQ(LLWU_IRQn);

	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);

	/*
	 *	Configure LLWU_P7 as low-leakage wakeup source.
	 *
	 *	See
	 *
	 *		Kinetis SDK v.1.1 API Reference Manual Chapter 33.
	 *	
	 *	and
	 *		KL03 Sub-Family Reference Manual, Rev. 4, August, 2014, Chapter 19.
	 *
	 *	Set to enable pin 7 (PTA0/IRQ0/LLWU_P7) as VLLx wakeup source, trigger on any edge.
	 */
	LLWU_HAL_SetExternalInputPinMode(LLWU_BASE, kLlwuExternalPinChangeDetect, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);

	USED(dummyread);
}

void
updateClockManagerToRunMode(uint8_t cmConfigMode)
{
	/*
	 *	If current config mode is RUN but CM is not, need to re-config it to RUN
	 */
	if ((cmConfigMode == CLOCK_CONFIG_INDEX_FOR_RUN) &&
		(CLOCK_SYS_GetCurrentConfiguration() != CLOCK_CONFIG_INDEX_FOR_RUN))
	{
		CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);
	}
}

void
updateClockManagerToVlprMode(uint8_t cmConfigMode)
{
	/*
	 *	If current config mode and CM are both RUN, need to re-config it to VLPR
	 */
	if ((cmConfigMode == CLOCK_CONFIG_INDEX_FOR_RUN) &&
		(CLOCK_SYS_GetCurrentConfiguration() != CLOCK_CONFIG_INDEX_FOR_VLPR))
	{
		CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_VLPR, kClockManagerPolicyForcible);
	}
}

void
update_clock_mode(uint8_t cmConfigMode)
{
	if (g_defaultClockConfigurations[cmConfigMode].mcgliteConfig.mcglite_mode == kMcgliteModeHirc48M)
	{
		CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);
	}
	else
	{
		updateClockManagerToRunMode(cmConfigMode);
	}
}
/*
 *	From KSDK power_manager_demo.c <<END
 */



WarpStatus
warpSetLowPowerMode(WarpPowerMode powerMode, uint32_t sleepSeconds)
{
	uint8_t				cmConfigMode = CLOCK_CONFIG_INDEX_FOR_RUN;
	power_manager_error_code_t	status;


	switch (powerMode)
	{
		case kWarpPowerModeWAIT:
		{
			if (POWER_SYS_GetCurrentMode() == kPowerManagerVlpr)
			{
				return kWarpStatusPowerTransitionErrorVlpr2Wait;
			}

			#if (WARP_BUILD_ENABLE_DEVRV8803C7)
				/*
				*	Program RV8803 external interrupt
				*/
				warpEnableI2Cpins();
				setRTCCountdownRV8803C7(sleepSeconds, kWarpRV8803ExtTD_1HZ, true /* interupt_enable */);
				warpDisableI2Cpins();

				gpioEnableWakeUp();
			#else
				/*
				 *	In Glaux, because we have the external clock going to RTC_CLKIN,
				 *	we can actually have the RTC active in stop modes too.
				 *
				 *	See footnote 5 of Table 7-2. "Module operation in low-power modes".
				 *
				 *	TODO: Need to test Warp variant of firmware on Glaux HW and see if
				 *	we are able to wake from VLLS0.
				 */
				gpioDisableWakeUp();
				setSleepRtcAlarm(sleepSeconds);
			#endif

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);


			/*
			 *	After the mode transition returns (perhaps via interrupt handler)
			 */


			/*
			 *	For now, always go to VLPR upon completion of prior mode
			 */
			CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_VLPR, kClockManagerPolicyForcible);
			

			if (status != kPowerManagerSuccess)
			{
				return kWarpStatusErrorPowerSysSetmode;
			}

			break;
		}

		case kWarpPowerModeSTOP:
		{
			if (POWER_SYS_GetCurrentMode() == kPowerManagerVlpr)
			{
				return kWarpStatusPowerTransitionErrorVlpr2Stop;
			}

			#if (WARP_BUILD_ENABLE_DEVRV8803C7)
				/*
				 *	Program RV8803 external interrupt
				 */
				warpEnableI2Cpins();
				setRTCCountdownRV8803C7(sleepSeconds, kWarpRV8803ExtTD_1HZ, true /* interupt_enable */);
				warpDisableI2Cpins();

				gpioEnableWakeUp();
			#else
				/*
				 *	In Glaux, because we have the external clock going to RTC_CLKIN,
				 *	we can actually have the RTC active in stop modes too.
				 *
				 *	See footnote 5 of Table 7-2. "Module operation in low-power modes".
				 *
				 *	TODO: Need to test Warp variant of firmware on Glaux HW and see if
				 *	we are able to wake from VLLS0.
				 */
				gpioDisableWakeUp();
				setSleepRtcAlarm(sleepSeconds);
			#endif

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

			/*
			 *	For now, always go to VLPR upon completion of prior mode
			 */
			CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_VLPR, kClockManagerPolicyForcible);

			if (status != kPowerManagerSuccess)
			{
				return kWarpStatusErrorPowerSysSetmode;
			}

			break;
		}

		case kWarpPowerModeVLPR:
		{
			if(kPowerManagerVlpr != POWER_SYS_GetCurrentMode())
			{
				if (	(cmConfigMode != CLOCK_CONFIG_INDEX_FOR_VLPR) &&
					(CLOCK_SYS_GetCurrentConfiguration() != CLOCK_CONFIG_INDEX_FOR_VLPR)
				)
				{
					CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_VLPR, kClockManagerPolicyForcible);
				}

				status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);
				if (status != kPowerManagerSuccess)
				{
					return kWarpStatusErrorPowerSysSetmode;
				}
			}
			else
			{
				return kWarpStatusPowerTransitionErrorVlpr2Vlpr;
			}

			break;
		}

		case kWarpPowerModeVLPW:
		{
			if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
			{
				return kWarpStatusPowerTransitionErrorRun2Vlpw;
			}

			#if (WARP_BUILD_ENABLE_DEVRV8803C7)
				/*
				*	Program RV8803 external interrupt
				*/
				warpEnableI2Cpins();
				setRTCCountdownRV8803C7(sleepSeconds, kWarpRV8803ExtTD_1HZ, true /* interupt_enable */);
				warpDisableI2Cpins();

				gpioEnableWakeUp();
			#else
				/*
				 *	In Glaux, because we have the external clock going to RTC_CLKIN,
				 *	we can actually have the RTC active in stop modes too.
				 *
				 *	See footnote 5 of Table 7-2. "Module operation in low-power modes".
				 *
				 *	TODO: Need to test Warp variant of firmware on Glaux HW and see if
				 *	we are able to wake from VLLS0.
				 */
				gpioDisableWakeUp();
				setSleepRtcAlarm(sleepSeconds);
			#endif

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

			if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
			{
				/*
				 *	For now, always go to VLPR upon completion of prior mode
				 */
				CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_VLPR, kClockManagerPolicyForcible);
			}

			if (status != kPowerManagerSuccess)
			{
				return kWarpStatusErrorPowerSysSetmode;
			}

			break;
		}

		case kWarpPowerModeVLPS:
		{
			#if (WARP_BUILD_ENABLE_DEVRV8803C7)
				/*
				 *	Program RV8803 external interrupt
				 */
				warpEnableI2Cpins();
				setRTCCountdownRV8803C7(sleepSeconds, kWarpRV8803ExtTD_1HZ, true /* interupt_enable */);
				warpDisableI2Cpins();

				gpioEnableWakeUp();
			#else
				/*
				 *	In Glaux, because we have the external clock going to RTC_CLKIN,
				 *	we can actually have the RTC active in stop modes too.
				 *
				 *	See footnote 5 of Table 7-2. "Module operation in low-power modes".
				 *
				 *	TODO: Need to test Warp variant of firmware on Glaux HW and see if
				 *	we are able to wake from VLLS0.
				 */
				gpioDisableWakeUp();
				setSleepRtcAlarm(sleepSeconds);
			#endif

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

			/*
			 *	After returning from RTC handler...
			 */

			if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
			{
				/*
				 *	For now, always go to VLPR upon completion of prior mode
				 */
				CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_VLPR, kClockManagerPolicyForcible);
				
			}

			if (status != kPowerManagerSuccess)
			{
				return kWarpStatusErrorPowerSysSetmode;
			}

			break;
		}

		case kWarpPowerModeRUN:
		{
			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);
			
			/*
			 *	In this case, we should return from POWER_SYS_SetMode() immediately
			 *	since we don't go to sleep.
			 */
			if (status != kPowerManagerSuccess)
			{
				return kWarpStatusErrorPowerSysSetmode;
			}
			else
			{
				CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);
			}

			break;
		}

		case kWarpPowerModeVLLS0:
		{
			#if (WARP_BUILD_ENABLE_DEVRV8803C7)
				/*
				 *	Program RV8803 external interrupt
				 */
				warpEnableI2Cpins();
				setRTCCountdownRV8803C7(sleepSeconds, kWarpRV8803ExtTD_1HZ, true /* interupt_enable */);
				warpDisableI2Cpins();

				/*
				 *	Since we wakeup via reset and we also turn these back on at reset:
				 */
				gpioEnableWakeUp();
			#else
				/*
				 *	In Glaux, because we have the external clock going to RTC_CLKIN,
				 *	we can actually have the RTC active in stop modes too.
				 *
				 *	See footnote 5 of Table 7-2. "Module operation in low-power modes".
				 *
				 *	TODO: Need to test Warp variant of firmware on Glaux HW and see if
				 *	we are able to wake from VLLS0.
				 */
				gpioDisableWakeUp();
				setSleepRtcAlarm(sleepSeconds);
			#endif

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

			/*
			 *	All the VLLSx sleeps can only wake up via a transition to
			 *	(soft) reset once their wakeup source fires. See, e.g., 
			 *	AN4503 Figure 11.  Therefore, if we get here, it must be that
			 *	POWER_SYS_SetMode() failed.
			 */
			return kWarpStatusErrorPowerSysSetmode;
		}


		case kWarpPowerModeVLLS1:
		{
			/*
			 *	TODO: this can be replaced using the internal RTC
			 */
			#if (WARP_BUILD_ENABLE_DEVRV8803C7)
				/*
				 *	Program RV8803 external interrupt
				 */
				warpEnableI2Cpins();
				setRTCCountdownRV8803C7(sleepSeconds, kWarpRV8803ExtTD_1HZ, true /* interupt_enable */);
				warpDisableI2Cpins();

				gpioEnableWakeUp();
			#else
				/*
				 *	In Glaux, because we have the external clock going to RTC_CLKIN,
				 *	we can actually have the RTC active in stop modes too.
				 *
				 *	See footnote 5 of Table 7-2. "Module operation in low-power modes".
				 *
				 *	TODO: Need to test Warp variant of firmware on Glaux HW and see if
				 *	we are able to wake from VLLS0.
				 */
				gpioDisableWakeUp();
				setSleepRtcAlarm(sleepSeconds);
			#endif

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

			/*
			 *	All the VLLSx sleeps can only wake up via a transition to
			 *	(soft) reset once their wakeup source fires. See, e.g., 
			 *	AN4503 Figure 11.  Therefore, if we get here, it must be that
			 *	POWER_SYS_SetMode() failed.
			 */
			return kWarpStatusErrorPowerSysSetmode;
		}

		case kWarpPowerModeVLLS3:
		{
			/*
			 *	TODO: this can be replaced using the internal RTC
			 */
			#if (WARP_BUILD_ENABLE_DEVRV8803C7)
				/*
				 *	Program RV8803 external interrupt
				 */
				warpEnableI2Cpins();
				setRTCCountdownRV8803C7(sleepSeconds, kWarpRV8803ExtTD_1HZ, true /* interupt_enable */);
				warpDisableI2Cpins();

				gpioEnableWakeUp();
			#else
				/*
				 *	In Glaux, because we have the external clock going to RTC_CLKIN,
				 *	we can actually have the RTC active in stop modes too.
				 *
				 *	See footnote 5 of Table 7-2. "Module operation in low-power modes".
				 *
				 *	TODO: Need to test Warp variant of firmware on Glaux HW and see if
				 *	we are able to wake from VLLS0.
				 */
				gpioDisableWakeUp();
				setSleepRtcAlarm(sleepSeconds);
			#endif

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

			/*
			 *	All the VLLSx sleeps can only wake up via a transition to
			 *	(soft) reset once their wakeup source fires. See, e.g., 
			 *	AN4503 Figure 11.  Therefore, if we get here, it must be that
			 *	POWER_SYS_SetMode() failed.
			 */
			return kWarpStatusErrorPowerSysSetmode;
		}

		default:
		{
			return kWarpStatusBadPowerModeSpecified;
		}
	}

	return kWarpStatusOK;
}
