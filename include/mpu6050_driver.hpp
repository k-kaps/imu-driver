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
#define ACCL_XOUT_L 0x3C
#define ACCL_YOUT_H 0x3D
#define ACCL_YOUT_L 0x3E
#define ACCL_ZOUT_H 0x3F
#define ACCL_ZOUT_L 0x40

// Gyroscope Registers
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define SELF_TEST_GYRO 0x1B
#define SELF_TEST_ACCL 0x1C

class MPU6050Driver {
public:
	MPU6050Driver();
	~MPU6050Driver();
	void initializeDriver();
	void readGyroscopeValues();
	void readAccelerometerValues();
	void selfTest();
private:
	int file_;
	bool init_;
	int16_t readValue(char reg_addr);
	void errorHandler(const std::string& message);
	bool checkInitialized();
};
