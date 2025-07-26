/**
 * INFO:
 * 	Contains Register Maps, Structs and Enums for the MPU6050 Driver
 */

// Init Registers
#define I2C_ADDRESS 0x68
#define I2C_ADAPTER_NUM 1
#define WHO_AM_I_REG 0x75
#define WAKE_UP_REG 0x6B

// Configuration registers
#define DLPF_CONFIG 0x1A
#define ACCL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B

// Accelerometer Registers
#define ACCL_XOUT_H 0x3B
#define ACCL_YOUT_H 0x3D
#define ACCL_ZOUT_H 0x3F

// Gyroscope Registers
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

#define TEMP_REG 0x41

#define GRAVITY 9.81
#define DELAY_TIME 200'000

// Enums for storing config
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

enum DlpfCFG {
	DLPF_CFG_0,
	DLPF_CFG_1,
	DLPF_CFG_2,
	DLPF_CFG_3,
	DLPF_CFG_4,
	DLPF_CFG_5,
	DLPF_CFG_6,
	DLPF_CFG_7
};

// Structs for Config, Accl, Gyro and IMU data
struct MPU6050Config {
        AcclFSR accl_fsr = A_FSR_2G;
	GyroFSR gyro_fsr = G_FSR_250;
	DlpfCFG dlpf_cfg = DLPF_CFG_0;
};

struct AcclData {
	double x;
	double y;
	double z;
};

struct GyroData {
	double x;
	double y;
	double z;
};

struct IMUData {
	AcclData accl;
	GyroData gyro;
};

struct IMUStamped {
	IMUData data;
	int64_t timestamp;
};

struct AcclStamped {
	AcclData data;
	int64_t timestamp;
};

struct GyroStamped {
	GyroData data;
	int64_t timestamp;
};

struct TempStamped {
	double data;
	int64_t timestamp;
};
