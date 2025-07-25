#include <mpu6050_driver.hpp>

int main() {
	MPU6050Config imu_config;
	imu_config.accl_fsr = A_FSR_2G;
	imu_config.gyro_fsr = G_FSR_250;

	MPU6050Driver imu = MPU6050Driver(imu_config);
	
	imu.init_driver();
	imu.read_accl();
    	imu.read_gyro();
	imu.read_temp();
}
