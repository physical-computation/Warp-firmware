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

#include "warp.h"


/*
 *	From KSDK power_manager_demo.c <<BEGIN>>>
 */

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

void
gpioDisableWakeUp(void)
{
#define BOARD_SW_LLWU_PIN            0
#define BOARD_SW_LLWU_BASE           PORTB_BASE
#define BOARD_SW_LLWU_IRQ_HANDLER    PORTB_IRQHandler
#define BOARD_SW_LLWU_IRQ_NUM        PORTB_IRQn
	
	// disables interrupt
	PORT_HAL_SetPinIntMode(BOARD_SW_LLWU_BASE, BOARD_SW_LLWU_PIN, kPortIntDisabled);
	INT_SYS_DisableIRQ(BOARD_SW_LLWU_IRQ_NUM);
}

void
updateClockManagerToRunMode(uint8_t cmConfigMode)
{
	// if current config mode is RUN but CM is not, need to re-config it to RUN
	if ((cmConfigMode == CLOCK_CONFIG_INDEX_FOR_RUN) &&
		(CLOCK_SYS_GetCurrentConfiguration() != CLOCK_CONFIG_INDEX_FOR_RUN))
	{
		CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);
	}
}

void
updateClockManagerToVlprMode(uint8_t cmConfigMode)
{
	// if current config mode and CM are both RUN, need to re-config it to VLPR
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
 *	From KSDK power_manager_demo.c <<END>>>
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

			gpioDisableWakeUp();
			setSleepRtcAlarm(sleepSeconds);
			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);


			/*
			 *	After the mode transition returns (perhaps via interrupt handler)
			 */

			// for now, always go to VLPR upon completion of prior mode
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

			gpioDisableWakeUp();
			setSleepRtcAlarm(sleepSeconds);

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

			// for now, always go to VLPR upon completion of prior mode
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

			gpioDisableWakeUp();
			setSleepRtcAlarm(sleepSeconds);

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

			if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
			{
				// for now, always go to VLPR upon completion of prior mode
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
			gpioDisableWakeUp();
			setSleepRtcAlarm(sleepSeconds);

			status = POWER_SYS_SetMode(powerMode, kPowerManagerPolicyAgreement);

			/*
			 *	After returning from RTC handler...
			 */

			if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
			{
				// for now, always go to VLPR upon completion of prior mode
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
#ifdef WARP_BUILD_ENABLE_DEVRV8803C7
			/*
			 *	program RV8803 external interrupt
			 */
			setRTCCountdownRV8803C7(sleepSeconds, TD_1HZ, true);
			/*
			 *	Turn off reset filter while in VLLSx Mode for reliable detection,
			 *	as the RV8803C7 interrupt self clears (in this mode) after 7ms
			 */
			BW_RCM_RPFC_RSTFLTSS(RCM_BASE, false);
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
#ifdef WARP_BUILD_ENABLE_DEVRV8803C7
			/*
			 *	program RV8803 external interrupt
			 */
			setRTCCountdownRV8803C7(sleepSeconds, TD_1HZ, true);
			/*
			 *	Turn off reset filter while in VLLSx Mode for reliable detection,
			 *	as the RV8803C7 interrupt self clears (in this mode) after 7ms
			 */
			BW_RCM_RPFC_RSTFLTSS(RCM_BASE, false);
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
#ifdef WARP_BUILD_ENABLE_DEVRV8803C7
			/*
			 *	program RV8803 external interrupt
			 */
			setRTCCountdownRV8803C7(sleepSeconds, TD_1HZ, true);
			/*
			 *	Turn off reset filter while in VLLSx Mode for reliable detection,
			 *	as the RV8803C7 interrupt self clears (in this mode) after 7ms
			 */
			BW_RCM_RPFC_RSTFLTSS(RCM_BASE, false);
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
