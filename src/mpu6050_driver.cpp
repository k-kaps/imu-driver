#include <mpu6050_driver.hpp>

MPU6050Driver::MPU6050Driver(){
	int file;
	char filename[20];

	snprintf(filename, sizeof(filename), "/dev/i2c-%d", I2C_ADAPTER_NUM);
	file = open(filename, O_RDWR);
	if (file < 0) {
		std::cerr << "ERRNO: " << errno << " - " << strerror(errno) << std::endl;
		exit(1);
	}
	if (ioctl(file, I2C_SLAVE, I2C_ADDRESS) < 0) {
		std::cerr << "ERRNO: " << errno << " - " << strerror(errno) << std::endl;
		exit(1);
	}

}

void MPU6050Driver::InitializeDriver() {
	
}
