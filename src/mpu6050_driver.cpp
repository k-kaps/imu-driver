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

	char dlpf_buf[2] = {DLPF_CONFIG, (config_.dlpf_cfg)};
	if(write(file_, dlpf_buf, sizeof(dlpf_buf)) != 2) {
		error_handler("Failed to set DLPF Configguration");
	}
}

// void MPU6050Driver::configure_self_test(bool on) {
// 	char cfg_byte = 0x00;
// 	if (on) {
// 		cfg_byte = 0xE0;
// 	}
	
// 	char accl_cfg[2] = {ACCL_CONFIG, cfg_byte};
// 	if (write(file_, accl_cfg, 2) != 2) {
// 		error_handler("Failed to toggle self test for Accelerometer");
// 	}

// 	char gyro_cfg[2] = {GYRO_CONFIG, cfg_byte};
// 	if (write(file_, gyro_cfg, 2) != 2) {
// 		error_handler("Failed to toggle self test for Gyroscope");
// 	}
// }

void MPU6050Driver::configure_fifo(FIFOSignals signals) {
	char fifo_en[2] = {FIFO_ENABLE, signals};
	if (write(file_, fifo_en, 2) != 2) {
		error_handler("Failed to configure FIFO");
	}
}

void MPU6050Driver::reset_fifo_buffer() {
	char fifo_reset[2] = {USER_CONTROL, 0x40};

	if (write(file_, fifo_reset, 2) != 2) {
		error_handler("Failed to reset FIFO Buffer");
	}
}

void MPU6050Driver::set_fifo_enabled(bool enable, FIFOSignals signals) {
	char fifo_byte = 0x00;
	if (!enable) {
		configure_fifo(FIFOSignals::NONE);
		reset_fifo_buffer();
	}
	else {
		configure_fifo(signals);
		fifo_byte = 0x40;
	}
	char usr_ctrl[2] = {USER_CONTROL, fifo_byte};
	if (write(file_, usr_ctrl, 2) != 2) {
		error_handler("Failed to toggle USER CONTROL");
	}
}


bool MPU6050Driver::read_accl(AcclStamped& accl_stamped) {
	if (read_accl(accl_stamped.data)) {
		accl_stamped.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>
			(std::chrono::system_clock::now().time_since_epoch()).count();
		return true;
	}
	return true;
}

bool MPU6050Driver::read_gyro(GyroStamped& gyro_stamped) {
	if (read_gyro(gyro_stamped.data)) {
		gyro_stamped.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>
			(std::chrono::system_clock::now().time_since_epoch()).count();
		return true;
	}
	return false;
}

bool MPU6050Driver::read_accl(AcclData& accl_data) {
	if (!check_init()) return false;

	uint8_t accl_hl_buf[6];
	read_reg(ACCL_XOUT_H, accl_hl_buf, 6);
	
	int16_t accl_raw_buf[3];
	unpack_hl_vals(accl_hl_buf, accl_raw_buf);
	process_raw_buf(accl_data, accl_raw_buf);

	return true;
}

bool MPU6050Driver::read_gyro(GyroData& gyro_data) {
	if (!check_init()) return false;

	uint8_t gyro_hl_buf[6];
	read_reg(GYRO_XOUT_H, gyro_hl_buf, 6);
	
	int16_t gyro_raw_buf[3];
	unpack_hl_vals(gyro_hl_buf, gyro_raw_buf);
	process_raw_buf(gyro_data, gyro_raw_buf);

	return true;
}

bool MPU6050Driver::read_imu(IMUStamped& imu_stamped) {
	if (read_imu(imu_stamped.data)) {
		imu_stamped.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>
			(std::chrono::system_clock::now().time_since_epoch()).count();
		return true;
	}
	return false;
}

bool MPU6050Driver::read_imu(IMUData& imu_data) {
	if (read_accl(imu_data.accl) && read_gyro(imu_data.gyro)) {
		return true;
	}
	return false;
}

void MPU6050Driver::process_raw_buf(AcclData& accl_data, int16_t* raw_buf) {
	accl_data.x = process_raw_accl_val(raw_buf[0]);
	accl_data.y = process_raw_accl_val(raw_buf[1]);
	accl_data.z = process_raw_accl_val(raw_buf[2]);
}

void MPU6050Driver::process_raw_buf(GyroData& gyro_data, int16_t* raw_buf) {
	gyro_data.x = process_raw_gyro_val(raw_buf[0]);
	gyro_data.y = process_raw_gyro_val(raw_buf[1]);
	gyro_data.z = process_raw_gyro_val(raw_buf[2]);
}

void MPU6050Driver::unpack_hl_vals(uint8_t* hl_buf, int16_t* raw_buf) {
	for(int i = 0; i < 3; i++) {
		raw_buf[i] = combine_hl_vals(&hl_buf[i*2]);
	}
}
int16_t MPU6050Driver::combine_hl_vals(uint8_t* buf) {
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

bool MPU6050Driver::read_temp(TempStamped& temp_stamped) {
	if (!check_init()) return false;

	uint8_t temp_buf[2];

	read_reg(TEMP_REG, temp_buf, 2);

	int16_t temp_raw = combine_hl_vals(temp_buf);

	temp_stamped.data = process_raw_temp_val(temp_raw);
	temp_stamped.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>
		(std::chrono::system_clock::now().time_since_epoch()).count();

	return true;
}

bool MPU6050Driver::get_fifo_status() {
	uint8_t fifo_status[1];
	read_reg(USER_CONTROL, fifo_status, 1);
	if ((fifo_status[1] >> 6) & 0x01) {
		return true;
	}
	else {
		return false;
	}
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
