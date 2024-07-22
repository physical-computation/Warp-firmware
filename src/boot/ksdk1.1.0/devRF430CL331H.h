
void        initRF430CL331H(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus  writeSensorRegisterRF430CL331H(uint16_t deviceRegister, uint16_t payload);
WarpStatus  configureSensorRegisterRF430CL331H(uint16_t payloadOP_Mode);
WarpStatus  readSensorRegisterRF430CL331H(uint16_t deviceRegister, int numberOfBytes);
WarpStatus  StatusRF430CL331H();
