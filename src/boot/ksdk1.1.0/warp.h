#include "fsl_spi_master_driver.h"

#define	min(x,y)	((x) < (y) ? (x) : (y))
#define	USED(x)		(void)(x)

typedef enum
{
	kWarpTypeMaskTemperature	= (1 <<  0),
	kWarpTypeMaskPressure		= (1 <<  1),
	kWarpTypeMaskHumidity		= (1 <<  2),
	kWarpTypeMaskC02Concentration	= (1 <<  3),

	kWarpTypeMaskInfrared		= (1 <<  4),
	kWarpTypeMaskColor		= (1 <<  5),

	kWarpTypeMaskAccelerationX	= (1 <<  6),
	kWarpTypeMaskAccelerationY	= (1 <<  7),
	kWarpTypeMaskAccelerationZ	= (1 <<  8),

	kWarpTypeMaskAngularRateX	= (1 <<  9),
	kWarpTypeMaskAngularRateY	= (1 << 10),
	kWarpTypeMaskAngularRateZ	= (1 << 11),

	kWarpTypeMaskMagneticX		= (1 << 12),
	kWarpTypeMaskMagneticY		= (1 << 13),
	kWarpTypeMaskMagneticZ		= (1 << 14),

	kWarpTypeMaskFMStationID	= (1 << 15),

	kWarpTypeMaskLambda450V		= (1 << 16),
	kWarpTypeMaskLambda500B		= (1 << 17),
	kWarpTypeMaskLambda550G		= (1 << 18),
	kWarpTypeMaskLambda570Y		= (1 << 19),
	kWarpTypeMaskLambda600O		= (1 << 20),
	kWarpTypeMaskLambda650R		= (1 << 21),
	
	kWarpTypeMaskLambda610R		= (1 << 22),
	kWarpTypeMaskLambda680S		= (1 << 23),
	kWarpTypeMaskLambda730T		= (1 << 24),
	kWarpTypeMaskLambda760U		= (1 << 25),
	kWarpTypeMaskLambda810V		= (1 << 26),
	kWarpTypeMaskLambda860W		= (1 << 27),
	
	kWarpTypeMaskTotalVOC		= (1 << 28),
	kWarpTypeMaskEquivalentCO2	= (1 << 29),


	/*
	 *	Always keep these two as the last items.
	 */
	kWarpTypeMaskTime,
	kWarpTypeMaskMax,
} WarpTypeMask;

typedef enum
{
	/*
	 *	Always keep this as the last item.
	 */
	kWarpSignalPrecisionMax
} WarpSignalPrecision;


typedef enum
{
	/*
	 *	Always keep this as the last item.
	 */
	kWarpSignalAccuracyMax
} WarpSignalAccuracy;

typedef enum
{
	/*
	 *	Always keep this as the last item.
	 */
	kWarpSignalReliabilityMax
} WarpSignalReliability;

typedef enum
{
	/*
	 *	Always keep this as the last item.
	 */
	kWarpSignalNoiseMax
} WarpSignalNoise;

typedef enum
{
	kWarpStatusOK				= 0,

	kWarpStatusDeviceNotInitialized,
	kWarpStatusDeviceCommunicationFailed,
	kWarpStatusBadDeviceCommand,

	/*
	 *	Generic comms error 
	 */
	kWarpStatusCommsError,

	/*
	 *	Power mode routines
	 */
	kWarpStatusPowerTransitionErrorVlpr2Wait,
	kWarpStatusPowerTransitionErrorVlpr2Stop,
	kWarpStatusPowerTransitionErrorRun2Vlpw,
	kWarpStatusPowerTransitionErrorVlpr2Vlpr,
	kWarpStatusErrorPowerSysSetmode,
	kWarpStatusBadPowerModeSpecified,


	/*
	 *	Always keep this as the last item.
	 */
	kWarpStatusMax
} WarpStatus;

typedef enum
{
	/*
	 *	NOTE: This order is depended on by POWER_SYS_SetMode()
	 *
	 *	See KSDK13APIRM.pdf Section 55.5.3
	 */
	kWarpPowerModeWAIT,
	kWarpPowerModeSTOP,
	kWarpPowerModeVLPR,
	kWarpPowerModeVLPW,
	kWarpPowerModeVLPS,
	kWarpPowerModeVLLS0,
	kWarpPowerModeVLLS1,
	kWarpPowerModeVLLS3,
	kWarpPowerModeRUN,
} WarpPowerMode;

typedef enum
{
	kWarpSensorADXL362,
	kWarpSensorMMA8451Q,
	kWarpSensorBME680,
	kWarpSensorBMX055accel,
	kWarpSensorBMX055gyro,
	kWarpSensorBMX055mag,
	kWarpSensorTMP006B,
	kWarpSensorMAG3110,
	kWarpSensorL3GD20H,
	kWarpSensorLPS25H,
	kWarpSensorTCS34725,
	kWarpSensorSI4705,
	kWarpSensorHDC1000,
	kWarpSensorSI7021,
	kWarpSensorAMG8834,
	kWarpSensorCCS811,
	kWarpSensorPAN1326,
	kWarpSensorAS7262,
	kWarpSensorAS7263,
	kWarpSensorSCD30,
} WarpSensorDevice;

typedef enum
{
	kWarpModeDisableAdcOnSleep	= (1 << 0),
} WarpModeMask;


typedef enum
{
	kWarpSizesI2cBufferBytes		= 4,
	kWarpSizesSpiBufferBytes		= 3,
	kWarpSizesBME680CalibrationValuesCount	= 41,
} WarpSizes;

typedef struct
{
	uint8_t			i2cAddress;
	WarpTypeMask		signalType;
	uint8_t			i2cBuffer[kWarpSizesI2cBufferBytes];

	WarpStatus		deviceStatus;
} WarpI2CDeviceState;

typedef enum
{
	kWarpSensorConfigurationRegisterMMA8451QF_SETUP			= 0x09,
	kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1		= 0x2A,

	kWarpSensorConfigurationRegisterMAG3110CTRL_REG1		= 0x10,
	kWarpSensorConfigurationRegisterMAG3110CTRL_REG2		= 0x11,

	kWarpSensorConfigurationRegisterHDC1000Configuration		= 0x02,

	kWarpSensorConfigurationRegisterAMG8834PCTL			= 0x00,
	kWarpSensorConfigurationRegisterAMG8834RST			= 0x01,
	kWarpSensorConfigurationRegisterAMG8834FPSC			= 0x02,

	kWarpSensorConfigurationRegisterCCS811MEAS_MODE			= 0x01,
	kWarpSensorConfigurationRegisterCCS811APP_START			= 0xF4,

	kWarpSensorConfigurationRegisterBMX055accelPMU_RANGE		= 0x0F,
	kWarpSensorConfigurationRegisterBMX055accelPMU_BW		= 0x10,
	kWarpSensorConfigurationRegisterBMX055accelPMU_LPW		= 0x11,
	kWarpSensorConfigurationRegisterBMX055accelPMU_LOW_POWER	= 0x12,
	kWarpSensorConfigurationRegisterBMX055accelACCD_HBW		= 0x13,
	kWarpSensorConfigurationRegisterBMX055magPowerCtrl		= 0x4B,
	kWarpSensorConfigurationRegisterBMX055magOpMode			= 0x4C,
	kWarpSensorConfigurationRegisterBMX055gyroRANGE			= 0x0F,
	kWarpSensorConfigurationRegisterBMX055gyroBW			= 0x10,
	kWarpSensorConfigurationRegisterBMX055gyroLPM1			= 0x11,
	kWarpSensorConfigurationRegisterBMX055gyroRATE_HBW		= 0x13,

	kWarpSensorConfigurationRegisterL3GD20HCTRL1			= 0x20,
	kWarpSensorConfigurationRegisterL3GD20HCTRL2			= 0x21,
	kWarpSensorConfigurationRegisterL3GD20HCTRL5			= 0x24,

	kWarpSensorConfigurationRegisterBME680Reset			= 0xE0,
	kWarpSensorConfigurationRegisterBME680Config			= 0x75,
	kWarpSensorConfigurationRegisterBME680Ctrl_Meas			= 0x74,
	kWarpSensorConfigurationRegisterBME680Ctrl_Hum			= 0x72,
	kWarpSensorConfigurationRegisterBME680Ctrl_Gas_1		= 0x71,
	kWarpSensorConfigurationRegisterBME680Ctrl_Gas_0		= 0x70,

	kWarpSensorConfigurationRegisterBME680CalibrationRegion1Start	= 0x89,
	kWarpSensorConfigurationRegisterBME680CalibrationRegion1End	= 0xA2,
	kWarpSensorConfigurationRegisterBME680CalibrationRegion2Start	= 0xE1,
	kWarpSensorConfigurationRegisterBME680CalibrationRegion2End	= 0xF2,
} WarpSensorConfigurationRegister;

typedef enum
{
	kWarpSensorOutputRegisterMMA8451QOUT_X_MSB			= 0x01,
	kWarpSensorOutputRegisterMMA8451QOUT_X_LSB			= 0x02,
	kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB			= 0x03,
	kWarpSensorOutputRegisterMMA8451QOUT_Y_LSB			= 0x04,
	kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB			= 0x05,
	kWarpSensorOutputRegisterMMA8451QOUT_Z_LSB			= 0x06,

	kWarpSensorOutputRegisterMAG3110OUT_X_MSB			= 0x01,
	kWarpSensorOutputRegisterMAG3110OUT_X_LSB			= 0x02,
	kWarpSensorOutputRegisterMAG3110OUT_Y_MSB			= 0x03,
	kWarpSensorOutputRegisterMAG3110OUT_Y_LSB			= 0x04,
	kWarpSensorOutputRegisterMAG3110OUT_Z_MSB			= 0x05,
	kWarpSensorOutputRegisterMAG3110OUT_Z_LSB			= 0x06,
	kWarpSensorOutputRegisterMAG3110DIE_TEMP			= 0x0F,

	kWarpSensorOutputRegisterHDC1000Temperature			= 0x00,
	kWarpSensorOutputRegisterHDC1000Humidity			= 0x01,

	kWarpSensorOutputRegisterAMG8834TTHL				= 0x0E,
	kWarpSensorOutputRegisterAMG8834TTHH				= 0x0F,
	kWarpSensorOutputRegisterAMG8834T01L				= 0x80,
	kWarpSensorOutputRegisterAMG8834T64H				= 0xFF,

	kWarpSensorOutputRegisterCCS811ALG_DATA				= 0x02,
	kWarpSensorOutputRegisterCCS811RAW_DATA				= 0x03,

	kWarpSensorOutputRegisterBMX055accelACCD_X_LSB			= 0x02,
	kWarpSensorOutputRegisterBMX055accelACCD_X_MSB			= 0x03,
	kWarpSensorOutputRegisterBMX055accelACCD_Y_LSB			= 0x04,
	kWarpSensorOutputRegisterBMX055accelACCD_Y_MSB			= 0x05,
	kWarpSensorOutputRegisterBMX055accelACCD_Z_LSB			= 0x06,
	kWarpSensorOutputRegisterBMX055accelACCD_Z_MSB			= 0x07,
	kWarpSensorOutputRegisterBMX055accelACCD_TEMP			= 0x08,
	kWarpSensorOutputRegisterBMX055gyroRATE_X_LSB			= 0x02,
	kWarpSensorOutputRegisterBMX055gyroRATE_X_MSB			= 0x03,
	kWarpSensorOutputRegisterBMX055gyroRATE_Y_LSB			= 0x04,
	kWarpSensorOutputRegisterBMX055gyroRATE_Y_MSB			= 0x05,
	kWarpSensorOutputRegisterBMX055gyroRATE_Z_LSB			= 0x06,
	kWarpSensorOutputRegisterBMX055gyroRATE_Z_MSB			= 0x07,
	kWarpSensorOutputRegisterBMX055magX_LSB				= 0x42,
	kWarpSensorOutputRegisterBMX055magX_MSB				= 0x43,
	kWarpSensorOutputRegisterBMX055magY_LSB				= 0x44,
	kWarpSensorOutputRegisterBMX055magY_MSB				= 0x45,
	kWarpSensorOutputRegisterBMX055magZ_LSB				= 0x46,
	kWarpSensorOutputRegisterBMX055magZ_MSB				= 0x47,
	kWarpSensorOutputRegisterBMX055magRHALL_LSB			= 0x48,
	kWarpSensorOutputRegisterBMX055magRHALL_MSB			= 0x49,

	kWarpSensorOutputRegisterL3GD20HOUT_TEMP			= 0x26,
	kWarpSensorOutputRegisterL3GD20HOUT_X_L				= 0x28,
	kWarpSensorOutputRegisterL3GD20HOUT_X_H				= 0x29,
	kWarpSensorOutputRegisterL3GD20HOUT_Y_L				= 0x2A,
	kWarpSensorOutputRegisterL3GD20HOUT_Y_H				= 0x2B,
	kWarpSensorOutputRegisterL3GD20HOUT_Z_L				= 0x2C,
	kWarpSensorOutputRegisterL3GD20HOUT_Z_H				= 0x2D,

	kWarpSensorOutputRegisterBME680press_msb			= 0x1F,
	kWarpSensorOutputRegisterBME680press_lsb			= 0x20,
	kWarpSensorOutputRegisterBME680press_xlsb			= 0x21,
	kWarpSensorOutputRegisterBME680temp_msb				= 0x22,
	kWarpSensorOutputRegisterBME680temp_lsb				= 0x23,
	kWarpSensorOutputRegisterBME680temp_xlsb			= 0x24,
	kWarpSensorOutputRegisterBME680hum_msb				= 0x25,
	kWarpSensorOutputRegisterBME680hum_lsb				= 0x26,
} WarpSensorOutputRegister;

typedef struct
{
	/*
	 *	For holding ksdk-based error codes
	 */
	spi_status_t		ksdk_spi_status;

	WarpTypeMask		signalType;

	uint8_t			spiSourceBuffer[kWarpSizesSpiBufferBytes];
	uint8_t			spiSinkBuffer[kWarpSizesSpiBufferBytes];
	WarpStatus		deviceStatus;
} WarpSPIDeviceState;

typedef struct
{
	WarpTypeMask		signalType;
	WarpStatus		deviceStatus;
} WarpUARTDeviceState;

typedef struct
{
	uint8_t	errorCount;
} WarpPowerManagerCallbackStructure;

typedef enum
{
	kWarpThermalChamberMemoryFillEvenComponent	= 0b00110011,
	kWarpThermalChamberMemoryFillOddComponent	= 0b11001100,
	kWarpThermalChamberMMA8451QOutputBufferSize	= 3,
	kWarpThermalChamberKL03MemoryFillBufferSize	= 200,
	kWarpThermalChamberBusyLoopCountOffset		= 65535,
	kWarpThermalChamberBusyLoopAdder		= 99,
	kWarpThermalChamberBusyLoopMutiplier		= 254,
} WarpThermalChamber;

typedef struct
{
	/*
	 *	Fill up the remaining memory space using an array
	 *	The size of the array is highly dependent on
	 *	the firmware code size
	 */
	uint8_t		memoryFillingBuffer[kWarpThermalChamberKL03MemoryFillBufferSize];
	uint8_t		outputBuffer[kWarpThermalChamberMMA8451QOutputBufferSize];
} WarpThermalChamberKL03MemoryFill;

WarpStatus	warpSetLowPowerMode(WarpPowerMode powerMode, uint32_t sleepSeconds);
void		enableI2Cpins(uint16_t pullupValue);
void		disableI2Cpins(void);
void		enableSPIpins(void);
void		disableSPIpins(void);
