#include <mpu6050_driver.hpp>

MPU6050Driver::MPU6050Driver() : 
	file_(-1), init_(false) {}

void MPU6050Driver::init_driver() {
	char filename[20];
	snprintf(filename, 19, "/dev/i2c-%d", I2C_ADAPTER_NUM);
	
	file_ = open(filename, O_RDWR);
	if (file_ < 0) {
		error_handler("Failed to open I2C adapter");
	}
	if (ioctl(file_, I2C_SLAVE, I2C_ADDRESS) < 0) {
		error_handler("Failed to set I2C slave address");
	}
	
	char buf[2] = {0x6B, 0x00};
	if (write(file_, buf, sizeof(buf)) < 0) {
		error_handler("Failer to wake up device");
	}
	
	uint8_t val[1];
	read_reg(WHO_AM_I_REG, val, 1);
	if (*val != 0x68) {
		error_handler("Failed WHO_AM_I reg test when initializing");
	}	
	init_ = true;
}

void MPU6050Driver::read_gyro() {
	if (!check_init()) return;

	// Read the Gyroscope values
	/*
	int16_t gyro_x = readValue(GYRO_XOUT_H);
	int16_t gyro_y = readValue(GYRO_YOUT_H);
	int16_t gyro_z = readValue(GYRO_ZOUT_H);
	*/
	// std::cout << "Gyroscope: " << gyro_x << " " << gyro_y << " " << gyro_z << std::endl;
}

void MPU6050Driver::read_accl() {
	if (!check_init()) return;

	uint8_t accl_x_buf[2];
	uint8_t accl_y_buf[2];
	uint8_t accl_z_buf[2];

	read_reg(ACCL_XOUT_H, accl_x_buf, 2);
	read_reg(ACCL_YOUT_H, accl_y_buf, 2);
	read_reg(ACCL_ZOUT_H, accl_z_buf, 2);

	int16_t accl_x = process_hl_buf(accl_x_buf);
	int16_t accl_y = process_hl_buf(accl_y_buf);
	int16_t accl_z = process_hl_buf(accl_z_buf);
	
	std::cout << "Accelerometer: " << accl_x << " " << accl_y << " " << accl_z << std::endl;
}

int16_t MPU6050Driver::process_hl_buf(uint8_t* buf) {
	int16_t processed_val = (buf[0] << 8) | (buf[1] & 0xFF);
	return processed_val;
}

void MPU6050Driver::read_reg(char reg_addr, uint8_t* out_buf, uint8_t size) {
	if (write(file_, &reg_addr, 1) != 1) {
		char error[35];
		snprintf(error, sizeof(error), "Error writing to register: %x", reg_addr);
		error_handler(error);
	}
	
	if (read(file_, out_buf, size) != size) {
		char error[35];
		snprintf(error, sizeof(error), "Error reading from register: %x", reg_addr);
		error_handler(error);
	}
}

void MPU6050Driver::error_handler(const std::string& error) {
	std::cout << error << std::endl;
	std::cout << "ERRNO : " << strerror(errno) << std::endl;
	exit(1);
}

bool MPU6050Driver::check_init() {
	if (!init_) {
		std::cout << "Sensor not initialized" << std::endl;
		return false;
	}
	return true;
}

MPU6050Driver::~MPU6050Driver() {
	close(file_);
}
