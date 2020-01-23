The directory ksdk1.1.0 is a stripped down version of the KSDK1.1.0, with the following file modified

	ksdk1.1.0/platform/drivers/src/i2c/fsl_i2c_master_driver.c

The FOPT byte has also been changed in the startup assumbly code, from 0x3D to 0x35. See

	ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S

The tree is stripped down to the minimal to support the Warp board.
