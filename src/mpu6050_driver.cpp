#include <mpu6050_driver.hpp>

MPU6050Driver::MPU6050Driver(){
	InitializeDriver();
}

void MPU6050Driver::InitializeDriver() {
	char filename[20];
	snprintf(filename, sizeof(filename), "/dev/i2c-%d", I2C_ADAPTER_NUM);
	file_ = open(filename, O_RDWR);
	if (file_ < 0) {
		ErrorHandler("Error Opening I2C Adapter File");
	}
	if (ioctl(file_, I2C_SLAVE, I2C_ADDRESS) < 0) {
		ErrorHandler("Failed to Set I2C Slave Address");
	}
}

void MPU6050Driver::ErrorHandler(const std::string& error) {
	std::cout << error << std::endl;
	std::cout << "ERRNO : " << strerror(errno) << std::endl;
	exit(1);
}
