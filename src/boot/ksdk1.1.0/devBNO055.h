void		initBNO055(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterBNO055(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterBNO055(uint8_t deviceRegister, uint8_t payload);
WarpStatus	configureSensorRegisterBNO055(uint8_t payloadOP_Mode, uint8_t payloadPWR_Mode);
void		printSensorDataBNO055(bool hexModeFlag);
WarpStatus	StateBNO055();
uint8_t		appendSensorDataBNO055(uint8_t* buf);


