extern "C" {
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}
#include <iostream>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <sys/ioctl.h>
#include <unistd.h>
#include <unordered_map>

#define I2C_ADDRESS 0x68
#define I2C_ADAPTER_NUM 1
#define WHO_AM_I_REG 0x75
#define WAKE_UP_REG 0x6B

// Accelerometer Registers
#define ACCL_CONFIG 0x1C
#define ACCL_XOUT_H 0x3B
#define ACCL_YOUT_H 0x3D
#define ACCL_ZOUT_H 0x3F

// Gyroscope Registers
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

#define TEMP_REG 0x41

#define GRAVITY 9.81

enum AcclFSR {
	A_FSR_2G,
	A_FSR_4G,
	A_FSR_8G,
	A_FSR_16G 
};

enum GyroFSR {
	G_FSR_250,
	G_FSR_500,
	G_FSR_1000,
	G_FSR_2000
};

struct MPU6050Config {
	AcclFSR accl_fsr = A_FSR_2G;
	GyroFSR gyro_fsr = G_FSR_250;
};

class MPU6050Driver {
public:
	MPU6050Driver();
	MPU6050Driver(MPU6050Config& config);
	~MPU6050Driver();
	
	void init_driver();
	void read_accl();
	void read_gyro();
	void read_temp();
	void self_test();
private:
	int file_;
	bool init_;
	MPU6050Config config_;
	std::unordered_map<int, float> accl_map_ = 
		{{AcclFSR::A_FSR_2G, 16384.0}, {AcclFSR::A_FSR_4G, 8192.0}, 
		{AcclFSR::A_FSR_8G, 4096.0}, {AcclFSR::A_FSR_16G, 2048.0}};
	std::unordered_map<int, float> gyro_map_ = 
		{{GyroFSR::G_FSR_250, 131.0}, {GyroFSR::G_FSR_500, 65.5}, 
		{GyroFSR::G_FSR_1000, 32.8}, {GyroFSR::G_FSR_2000, 16.4}};
	int16_t combine_buf_vals(uint8_t* buf);
	
	double process_raw_accl_val(int16_t accl_val);
	double process_raw_gyro_val(int16_t gyro_val);
	double process_raw_temp_val(int16_t temp_val);

	void configure_imu();
	void read_reg(char reg_addr, uint8_t* out_buf, uint8_t size);
	
	void error_handler(const std::string& message);
	bool check_init();
};
