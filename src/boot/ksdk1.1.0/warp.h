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



WarpStatus	warpSetLowPowerMode(WarpPowerMode powerMode, uint32_t sleepSeconds);
void		enableI2Cpins(uint16_t pullupValue);
void		disableI2Cpins(void);
void		enableSPIpins(void);
void		disableSPIpins(void);
