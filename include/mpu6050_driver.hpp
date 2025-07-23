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

#define I2C_ADDRESS 0x68
#define I2C_ADAPTER_NUM 1

// Accelerometer Registers
#define ACCL_XOUT_H 0x3B
#define ACCL_YOUT_H 0x3D
#define ACCL_ZOUT_H 0x3F

// Gyroscope Registers
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

#define SELF_TEST_GYRO 0x1B
#define SELF_TEST_ACCL 0x1C
#define WHO_AM_I_REG 0x75

class MPU6050Driver {
public:
	MPU6050Driver();
	~MPU6050Driver();
	void init_driver();
	void read_gyro();
	void read_accl();
	void self_test();
private:
	int file_;
	bool init_;
	int16_t process_hl_buf(uint8_t* buf);
	void read_reg(char reg_addr, uint8_t* out_buf, uint8_t size);
	void error_handler(const std::string& message);
	bool check_init();
};
