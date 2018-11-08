# Warp-adapted
For private use only. *Youchao Wang*

***Important: Should you wish to fully appreciate Warp Firmware, PLEASE DO NOT USE THE CONTENTS IN THIS REPOSITORY***

Baseline firmware [Warp firmware](https://github.com/physical-computation/Warp-firmware) for the [Warp hardware](https://github.com/physical-computation/Warp-hardware) platform.


## Note for MMA8451Q 3-axis accelerometer
In `Warp-firmware` the I2C address was configured as `0x1C`.

        Line 1655: initMMA8451Q(	0x1C	/* i2cAddress */,	&deviceMMA8451QState	);	
        
Unfortunately, this is not the case for `FRDMKL03Z platform`, as `SA0` address selection pin on `MMA8451Q` is high. 

The correct I2C address should be `0x1D`.
