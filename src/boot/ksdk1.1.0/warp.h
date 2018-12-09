#define	min(x,y)	((x) < (y) ? (x) : (y))

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


typedef struct
{
	uint8_t			i2cAddress;
	WarpTypeMask		signalType;

	/*
	 *	TODO: for now, magic numbers for buffer sizes
	 */
	uint8_t			i2cBuffer[2];

	WarpStatus		deviceStatus;
} WarpI2CDeviceState;

typedef enum
{
	kWarpSensorMMA8451QF_SETUP	= 0x09,
	kWarpSensorMMA8451QCTRL_REG1	= 0x2A,
	kWarpSensorMAG3110CTRL_REG1	= 0x10,
	kWarpSensorMAG3110CTRL_REG2	= 0x11,
	kWarpSensorHDC1000Configuration	= 0x02,
	kWarpSensorAMG8834Configuration	= 0x01,
	kWarpSensorAMG8834FrameRate	= 0x02,
	kWarpSensorCCS811MEAS_MODE	= 0x01,
	kWarpSensorCCS811APP_START	= 0xF4,
	kWarpSensorBMX055accelPMU_RANGE	= 0x0F,
	kWarpSensorBMX055accelPMU_BW	= 0x10,
	kWarpSensorBMX055accelPMU_LPW	= 0x11,
	kWarpSensorBMX055accelPMU_LOW_POWER	= 0x12,
	kWarpSensorBMX055accelACCD_HBW	= 0x13,
	kWarpSensorBMX055magPowerCtrl	= 0x4B,
	kWarpSensorBMX055magOpMode	= 0x4C,
	kWarpSensorBMX055gyroRANGE	= 0x0F,
	kWarpSensorBMX055gyroBW		= 0x10,
	kWarpSensorBMX055gyroLPM1	= 0x11,
	kWarpSensorBMX055gyroRATE_HBW	= 0x13,
	kWarpSensorL3GD20HCTRL1		= 0x20,
	kWarpSensorL3GD20HCTRL2		= 0x21,
	kWarpSensorL3GD20HCTRL5		= 0x24,
	kWarpSensorBME680Ctrl_Meas	= 0x74,
	kWarpSensorBME680Ctrl_Hum	= 0x72,
	kWarpSensorBME680Config		= 0x75,
	kWarpSensorBME680Ctrl_Gas_1	= 0x71,
	kWarpSensorBME680Gas_Wait_0	= 0x64,
} WarpI2CDeviceConfigurationRegister;

typedef enum
{
	kWarpSensorMMA8451QOUT_X_MSB	= 0x01,
	kWarpSensorMMA8451QOUT_X_LSB	= 0x02,
	kWarpSensorMMA8451QOUT_Y_MSB	= 0x03,
	kWarpSensorMMA8451QOUT_Y_LSB	= 0x04,
	kWarpSensorMMA8451QOUT_Z_MSB	= 0x05,
	kWarpSensorMMA8451QOUT_Z_LSB	= 0x06,
	kWarpSensorMAG3110OUT_X_MSB	= 0x01,
	kWarpSensorMAG3110OUT_X_LSB	= 0x02,
	kWarpSensorMAG3110OUT_Y_MSB	= 0x03,
	kWarpSensorMAG3110OUT_Y_LSB	= 0x04,
	kWarpSensorMAG3110OUT_Z_MSB	= 0x05,
	kWarpSensorMAG3110OUT_Z_LSB	= 0x06,
	kWarpSensorMAG3110DIE_TEMP	= 0x0F,
	kWarpSensorHDC1000Temperature	= 0x00,
	kWarpSensorHDC1000Humidity	= 0x01,
	kWarpSensorAMG8834TTHL		= 0x0E,
	kWarpSensorAMG8834TTHH		= 0x0F,
	kWarpSensorAMG8834T01L		= 0x80,
	kWarpSensorAMG8834T64H		= 0xFF,
	kWarpSensorCCS811RAW_DATA	= 0x03,
	kWarpSensorBMX055accelACCD_X_LSB= 0x02,
	kWarpSensorBMX055accelACCD_X_MSB= 0x03,
	kWarpSensorBMX055accelACCD_Y_LSB= 0x04,
	kWarpSensorBMX055accelACCD_Y_MSB= 0x05,
	kWarpSensorBMX055accelACCD_Z_LSB= 0x06,
	kWarpSensorBMX055accelACCD_Z_MSB= 0x07,
	kWarpSensorBMX055accelACCD_TEMP	= 0x08,
	kWarpSensorBMX055gyroRATE_X_LSB	= 0x02,
	kWarpSensorBMX055gyroRATE_X_MSB	= 0x03,
	kWarpSensorBMX055gyroRATE_Y_LSB	= 0x04,
	kWarpSensorBMX055gyroRATE_Y_MSB	= 0x05,
	kWarpSensorBMX055gyroRATE_Z_LSB	= 0x06,
	kWarpSensorBMX055gyroRATE_Z_MSB	= 0x07,
	kWarpSensorBMX055magX_LSB	= 0x42,
	kWarpSensorBMX055magX_MSB	= 0x43,
	kWarpSensorBMX055magY_LSB	= 0x44,
	kWarpSensorBMX055magY_MSB	= 0x45,
	kWarpSensorBMX055magZ_LSB	= 0x46,
	kWarpSensorBMX055magZ_MSB	= 0x47,
	kWarpSensorBMX055magRHALL_LSB	= 0x48,
	kWarpSensorBMX055magRHALL_MSB	= 0x49,
	kWarpSensorL3GD20HOUT_TEMP	= 0x26,
	kWarpSensorL3GD20HOUT_X_L	= 0x28,
	kWarpSensorL3GD20HOUT_X_H	= 0x29,
	kWarpSensorL3GD20HOUT_Y_L	= 0x2A,
	kWarpSensorL3GD20HOUT_Y_H	= 0x2B,
	kWarpSensorL3GD20HOUT_Z_L	= 0x2C,
	kWarpSensorL3GD20HOUT_Z_H	= 0x2D,
	kWarpSensorBME680press_msb	= 0x1F,
	kWarpSensorBME680press_lsb	= 0x20,
	kWarpSensorBME680press_xlsb	= 0x21,
	kWarpSensorBME680temp_msb	= 0x22,
	kWarpSensorBME680temp_lsb	= 0x23,
	kWarpSensorBME680temp_xlsb	= 0x24,
	kWarpSensorBME680hum_msb	= 0x25,
	kWarpSensorBME680hum_lsb	= 0x26,
} WarpI2CDeviceDataOutputRegister;

typedef struct
{
	/*
	 *	For holding ksdk-based error codes
	 */
	spi_status_t		ksdk_spi_status;

	WarpTypeMask		signalType;

	/*
	 *	TODO: for now, magic numbers for buffer sizes
	 */
	uint8_t			spiSourceBuffer[3];
	uint8_t			spiSinkBuffer[3];
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
