#include <mpu6050_driver.hpp>

MPU6050Driver::MPU6050Driver() : 
	file_(-1), init_(false), config_(){}

MPU6050Driver::MPU6050Driver(MPU6050Config& config) :
	file_(-1), init_(false), config_(config) {}

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
	
	char buf[2] = {WAKE_UP_REG, 0x00};
	if (write(file_, buf, sizeof(buf)) != 2) {
		error_handler("Failed to wake up device");
	}
	
	uint8_t val[1];
	read_reg(WHO_AM_I_REG, val, 1);
	if (*val != 0x68) {
		error_handler("Failed WHO_AM_I reg test when initializing");
	}
	
	configure_imu();
	
	init_ = true;
}

void MPU6050Driver::configure_imu() {
	char gyro_buf[2] = {GYRO_CONFIG, (config_.gyro_fsr << 3)};
	if(write(file_, gyro_buf, sizeof(gyro_buf)) != 2) {
		error_handler("Failed to set Gyroscope Configuration");
	}

	char accl_buf[2] = {ACCL_CONFIG, (config_.accl_fsr << 3)};
	if(write(file_, accl_buf, sizeof(accl_buf)) != 2) {
		error_handler("Failed to set Accelerometer Configuration");
	}
}

void MPU6050Driver::read_gyro() {
	if (!check_init()) return;

	uint8_t gyro_x_buf[2];
	uint8_t gyro_y_buf[2];
	uint8_t gyro_z_buf[2];

	read_reg(GYRO_XOUT_H, gyro_x_buf, 2);
	read_reg(GYRO_YOUT_H, gyro_y_buf, 2);
	read_reg(GYRO_ZOUT_H, gyro_z_buf, 2);

	int16_t gyro_raw_x = combine_buf_vals(gyro_x_buf);
	int16_t gyro_raw_y = combine_buf_vals(gyro_y_buf);
	int16_t gyro_raw_z = combine_buf_vals(gyro_z_buf);

	double gyro_x = process_raw_gyro_val(gyro_raw_x);
	double gyro_y = process_raw_gyro_val(gyro_raw_y);
	double gyro_z = process_raw_gyro_val(gyro_raw_z);
	
	std::cout << "Gyroscope: " << gyro_x << " " << gyro_y << " " << gyro_z << std::endl;
}

void MPU6050Driver::read_accl() {
	if (!check_init()) return;

	uint8_t accl_x_buf[2];
	uint8_t accl_y_buf[2];
	uint8_t accl_z_buf[2];

	read_reg(ACCL_XOUT_H, accl_x_buf, 2);
	read_reg(ACCL_YOUT_H, accl_y_buf, 2);
	read_reg(ACCL_ZOUT_H, accl_z_buf, 2);

	int16_t accl_raw_x = combine_buf_vals(accl_x_buf);
	int16_t accl_raw_y = combine_buf_vals(accl_y_buf);
	int16_t accl_raw_z = combine_buf_vals(accl_z_buf);

	double accl_x = process_raw_accl_val(accl_raw_x);
	double accl_y = process_raw_accl_val(accl_raw_y);
	double accl_z = process_raw_accl_val(accl_raw_z);
	
	std::cout << "Accelerometer: " << accl_x << " " << accl_y << " " << accl_z << std::endl;
}

void MPU6050Driver::read_temp() {
	if (!check_init()) return;

	uint8_t temp_buf[2];

	read_reg(TEMP_REG, temp_buf, 2);

	int16_t temp_raw = combine_buf_vals(temp_buf);

	double temp = process_raw_temp_val(temp_raw);

	std::cout << "Temperature: " << temp << std::endl;
}

int16_t MPU6050Driver::combine_buf_vals(uint8_t* buf) {
	int16_t processed_val = (buf[0] << 8) | (buf[1] & 0xFF);
	return processed_val;
}

double MPU6050Driver::process_raw_accl_val(int16_t accl_val) {
	double accl_mps2 = accl_val * GRAVITY / accl_map_[config_.accl_fsr];
	return accl_mps2;
}

double MPU6050Driver::process_raw_gyro_val(int16_t gyro_val) {
	double gyro_degps = gyro_val / gyro_map_[config_.gyro_fsr];
	return gyro_degps;
}

double MPU6050Driver::process_raw_temp_val(int16_t temp_val) {
	double temp_C = temp_val/340.0 + 36.53;
	return temp_C;
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
	std::cerr << error << std::endl;
	std::cerr << "ERRNO : " << strerror(errno) << std::endl;
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
