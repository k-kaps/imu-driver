#include <mpu6050_driver.hpp>

MPU6050Driver::MPU6050Driver() : 
	file_(-1), init_(false) {}

void MPU6050Driver::initializeDriver() {
	char filename[20];
	snprintf(filename, 19, "/dev/i2c-%d", I2C_ADAPTER_NUM);
	
	file_ = open(filename, O_RDWR);
	if (file_ < 0) {
		errorHandler("Failed to open I2C adapter");
	}

	if (ioctl(file_, I2C_SLAVE, I2C_ADDRESS) < 0) {
		errorHandler("Failed to set I2C slave address");
	}
	char buf[2] = {0x6B, 0x00};
	if (write(file_, buf, sizeof(buf)) < 0) {
		errorHandler("Failer to wake up device");
	}
	init_ = true;
}

void MPU6050Driver::selfTest() {
	if (!checkInitialized()) return;

	// Do the self test
}

void MPU6050Driver::readGyroscopeValues() {
	if (!checkInitialized()) return;

	// Read the Gyroscope values
	int16_t gyro_x = readValue(GYRO_XOUT_H);
	int16_t gyro_y = readValue(GYRO_YOUT_H);
	int16_t gyro_z = readValue(GYRO_ZOUT_H);
}

void MPU6050Driver::readAccelerometerValues() {
	if (!checkInitialized()) return;

	// Read the accelerometer values
	int16_t accl_x = readValue(ACCL_XOUT_H);	
	int16_t accl_y = readValue(ACCL_YOUT_H);	
	int16_t accl_z = readValue(ACCL_ZOUT_H);
}

int16_t MPU6050Driver::readValue(char reg_addr) {
	if (write(file_, &reg_addr, 1) != 1) {
		char error[35];
		snprintf(error, sizeof(error), "Error writing to register: %x", reg_addr);
		errorHandler(error);
	}
	int8_t buf[2] = {0, 0};
	if (read(file_, buf, 2) != 2) {
		char error[35];
		snprintf(error, sizeof(error), "Error reading from register: %x", reg_addr);
		errorHandler(error);
	}
	int16_t val = ((buf[0]) << 8) | (buf[1] & 0xFF); 
	std::cout << val << std::endl;
	return val;
}

void MPU6050Driver::errorHandler(const std::string& error) {
	std::cout << error << std::endl;
	std::cout << "ERRNO : " << strerror(errno) << std::endl;
	exit(1);
}

bool MPU6050Driver::checkInitialized() {
	if (!init_) {
		std::cout << "Sensor not initialized" << std::endl;
		return false;
	}
	return true;
}

MPU6050Driver::~MPU6050Driver() {
	close(file_);
}
