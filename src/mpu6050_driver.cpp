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

void MPU6050Driver::toggle_self_test_cfg(bool on) {
	char cfg_byte = 0x00;
	if (on) {
		cfg_byte = 0xE0;
	}
	
	char accl_cfg[2] = {ACCL_CONFIG, cfg_byte};
	if (write(file_, accl_cfg, 2) != 2) {
		error_handler("Failed to toggle self test for Accelerometer");
	}

	char gyro_cfg[2] = {GYRO_CONFIG, cfg_byte};
	if (write(file_, gyro_cfg, 2) != 2) {
		error_handler("Failed to toggle self test for Gyroscope");
	}
}

void MPU6050Driver::toggle_fifo_cfg(bool on) {
	char fifo_cfg = 0x00;
	if (on) {
		fifo_cfg = 0x78;
	}
	
	char fifo_en[2] = {FIFO_ENABLE, fifo_cfg};
	if (write(file_, fifo_en, 2) != 2) {
		error_handler("Failed to toggle FIFO ENABLE");
	}
}

void MPU6050Driver::reset_fifo() {
	char fifo_reset[2] = {USER_CONTROL, 0x40};

	if (write(file_, fifo_reset, 2) != 2) {
		error_handler("Failed to reset FIFO");
	}
}

void MPU6050Driver::toggle_fifo(bool on) {
	char fifo_byte = 0x00;
	if (!on) {
		toggle_fifo_cfg(false);
		reset_fifo();
	}
	if (on) {
		toggle_fifo_cfg(true);
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

	uint8_t accl_x_buf[2];
	uint8_t accl_y_buf[2];
	uint8_t accl_z_buf[2];

	read_reg(ACCL_XOUT_H, accl_x_buf, 2);
	read_reg(ACCL_YOUT_H, accl_y_buf, 2);
	read_reg(ACCL_ZOUT_H, accl_z_buf, 2);

	int16_t accl_raw_x = combine_buf_vals(accl_x_buf);
	int16_t accl_raw_y = combine_buf_vals(accl_y_buf);
	int16_t accl_raw_z = combine_buf_vals(accl_z_buf);

	accl_data.x = process_raw_accl_val(accl_raw_x);
	accl_data.y = process_raw_accl_val(accl_raw_y);
	accl_data.z = process_raw_accl_val(accl_raw_z);

	return true;
}

bool MPU6050Driver::read_gyro(GyroData& gyro_data) {
	if (!check_init()) return false;

	uint8_t gyro_x_buf[2];
	uint8_t gyro_y_buf[2];
	uint8_t gyro_z_buf[2];

	read_reg(GYRO_XOUT_H, gyro_x_buf, 2);
	read_reg(GYRO_YOUT_H, gyro_y_buf, 2);
	read_reg(GYRO_ZOUT_H, gyro_z_buf, 2);
	
	int16_t gyro_raw_x = combine_buf_vals(gyro_x_buf);
	int16_t gyro_raw_y = combine_buf_vals(gyro_y_buf);
	int16_t gyro_raw_z = combine_buf_vals(gyro_z_buf);

	gyro_data.x = process_raw_gyro_val(gyro_raw_x);
	gyro_data.y = process_raw_gyro_val(gyro_raw_y);
	gyro_data.z = process_raw_gyro_val(gyro_raw_z);

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

bool MPU6050Driver::read_temp(TempStamped& temp_stamped) {
	if (!check_init()) return false;

	uint8_t temp_buf[2];

	read_reg(TEMP_REG, temp_buf, 2);

	int16_t temp_raw = combine_buf_vals(temp_buf);

	temp_stamped.data = process_raw_temp_val(temp_raw);
	temp_stamped.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>
		(std::chrono::system_clock::now().time_since_epoch()).count();

	return true;
}

bool MPU6050Driver::check_fifo_on() {
	uint8_t fifo_status[1];
	read_reg(USER_CONTROL, fifo_status, 1);
	if ((fifo_status[1] >> 6) & 0x01) {
		return true;
	}
	else {
		return false;
	}
}


bool MPU6050Driver::get_change_from_ft(AcclData& diff, AcclData& ft) {
	AcclData percent_resp = diff.str_percent(ft);
	std::cout << "ACCL" << std::endl;
	std::cout << percent_resp.x << " " << percent_resp.y << " " << percent_resp.z << std::endl;
	if (percent_resp.x > STR_THRESH || percent_resp.y > STR_THRESH || percent_resp.z > STR_THRESH) {
		return false;
	}	
	return true;
}

bool MPU6050Driver::get_change_from_ft(GyroData& diff, GyroData& ft) {
	GyroData percent_resp = diff.str_percent(ft);
	std::cout << "GYRO" << std::endl;
	std::cout << percent_resp.x << " " << percent_resp.y << " " << percent_resp.z << std::endl;
	if (percent_resp.x > STR_THRESH || percent_resp.y > STR_THRESH || percent_resp.z > STR_THRESH) {
		return false;
	}
	return true;
}

bool MPU6050Driver::compute_ft(AcclData& accl_data, GyroData& gyro_data) {
	uint8_t test_vals[4];
	read_reg(SELF_TEST, test_vals, 4);

	uint8_t xg_test = test_vals[0] & 0x1F;
	uint8_t yg_test = test_vals[1] & 0x1F;
	uint8_t zg_test = test_vals[2] & 0x1F;

	uint8_t xa_test = ((test_vals[0] >> 3) & 0x1C) | ((test_vals[3] >> 4) & 0x03);
	uint8_t ya_test = ((test_vals[1] >> 3) & 0x1C) | ((test_vals[3] >> 4) & 0x03);
	uint8_t za_test = ((test_vals[2] >> 3) & 0x1C) | ((test_vals[3] >> 4) & 0x03);

	accl_data.x = 4096 * 0.34 * pow((0.9/0.34), (xa_test - 1)/(30));
	accl_data.y = 4096 * 0.34 * pow((0.9/0.34), (ya_test - 1)/(30));
	accl_data.z = 4096 * 0.34 * pow((0.9/0.34), (za_test - 1)/(30));
	
	gyro_data.x = 25 * 131 * pow(1.046, (xg_test - 1));
	gyro_data.x = -25 * 131 * pow(1.046, (yg_test - 1));
	gyro_data.x = 25 * 131 * pow(1.046, (zg_test - 1));

	return true;
}

bool MPU6050Driver::self_test() {
	if (!check_init()) return false;
	if (check_fifo_on()) toggle_fifo(false);

	AcclData accl_st_on;
	AcclData accl_st_off;
	AcclData accl_ft;

	GyroData gyro_st_on;
	GyroData gyro_st_off;
	GyroData gyro_ft;

	toggle_self_test_cfg(true);
	usleep(DELAY_TIME);
	if (!compute_ft(accl_ft, gyro_ft)) return false;

	uint8_t count[2];
	toggle_fifo(true);
	usleep(DELAY_TIME);

	uint16_t num_samples = 0;
	while (num_samples < 60) {
		read_reg(FIFO_COUNT, count, 2);
		num_samples = combine_buf_vals(count);
	}
	
	toggle_self_test_cfg(false);
	uint8_t fifo_buf[12];
	for (int i = 0; i < 60; i = i + 12) {
		read_reg(FIFO_RW, fifo_buf, 12);
		
		int16_t accl_raw_x = combine_buf_vals(&fifo_buf[0]);
		int16_t accl_raw_y = combine_buf_vals(&fifo_buf[2]);
		int16_t accl_raw_z = combine_buf_vals(&fifo_buf[4]);
		int16_t gyro_raw_x = combine_buf_vals(&fifo_buf[6]);
		int16_t gyro_raw_y = combine_buf_vals(&fifo_buf[8]);
		int16_t gyro_raw_z = combine_buf_vals(&fifo_buf[10]);

		accl_st_on.x += process_raw_accl_val(accl_raw_x); 
		accl_st_on.y += process_raw_accl_val(accl_raw_y); 
		accl_st_on.z += process_raw_accl_val(accl_raw_z); 
		gyro_st_on.x += process_raw_gyro_val(gyro_raw_x); 
		gyro_st_on.y += process_raw_gyro_val(gyro_raw_y); 
		gyro_st_on.z += process_raw_gyro_val(gyro_raw_z);	
	}

	accl_st_on.x = accl_st_on.x/5;
	accl_st_on.y = accl_st_on.y/5;
	accl_st_on.z = accl_st_on.z/5;
	gyro_st_on.x = gyro_st_on.x/5;
	gyro_st_on.y = gyro_st_on.y/5;
	gyro_st_on.z = gyro_st_on.z/5;

	toggle_fifo(false);
	usleep(DELAY_TIME);

	toggle_fifo(true);
	usleep(DELAY_TIME);

	while (num_samples < 60) {
		read_reg(FIFO_COUNT, count, 2);
		num_samples = combine_buf_vals(count);
	}
	
	for (int i = 0; i < 60; i = i + 12) {
		read_reg(FIFO_RW, fifo_buf, 12);
		
		int16_t accl_raw_x = combine_buf_vals(&fifo_buf[0]);
		int16_t accl_raw_y = combine_buf_vals(&fifo_buf[2]);
		int16_t accl_raw_z = combine_buf_vals(&fifo_buf[4]);
		int16_t gyro_raw_x = combine_buf_vals(&fifo_buf[6]);
		int16_t gyro_raw_y = combine_buf_vals(&fifo_buf[8]);
		int16_t gyro_raw_z = combine_buf_vals(&fifo_buf[10]);

		accl_st_off.x += process_raw_accl_val(accl_raw_x); 
		accl_st_off.y += process_raw_accl_val(accl_raw_y); 
		accl_st_off.z += process_raw_accl_val(accl_raw_z); 
		gyro_st_off.x += process_raw_gyro_val(gyro_raw_x); 
		gyro_st_off.y += process_raw_gyro_val(gyro_raw_y); 
		gyro_st_off.z += process_raw_gyro_val(gyro_raw_z);	
	}

	accl_st_off.x = accl_st_off.x/5;
	accl_st_off.y = accl_st_off.y/5;
	accl_st_off.z = accl_st_off.z/5;
	gyro_st_off.x = gyro_st_off.x/5;
	gyro_st_off.y = gyro_st_off.y/5;
	gyro_st_off.z = gyro_st_off.z/5;
	
	toggle_fifo(false);
	
	AcclData diff_accl;
	GyroData diff_gyro;

	diff_accl = accl_st_on - accl_st_off;
	diff_gyro = gyro_st_on - gyro_st_off;

	get_change_from_ft(diff_accl, accl_ft);
	get_change_from_ft(diff_gyro, gyro_ft);
       	std::cout << "Failed Self Test" << std::endl;
	return false;
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
