#include <mpu6050_driver.hpp>

MPU6050Driver::MPU6050Driver(){
	int file;
	char filename[20];

	snprintf(filename, 19, "/dev/i2c-%d", I2C_ADAPTER_NUM);
	file = open(filename, O_RDWR);
}

void MPU6050Driver::InitializeDriver() {
	std::cout << "Initialize Driver" << std::endl;
}