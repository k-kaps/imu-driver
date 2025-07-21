#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <iostream>
#include <fcntl.h>

#define I2C_ADDRESS 0x68
#define I2C_ADAPTER_NUM 1

class MPU6050Driver {
public:
	MPU6050Driver();
private:
	void InitializeDriver();
};
