#include <mpu6050_driver.hpp>

int main() {
	MPU6050Config imu_config;
	imu_config.accl_fsr = A_FSR_8G;
	imu_config.gyro_fsr = G_FSR_250;

	MPU6050Driver imu = MPU6050Driver(imu_config);
	
	imu.init_driver();

	AcclStamped accl_stamped {};
	if(imu.read_accl(accl_stamped)) {
		std::cout << "Accelerometer Data : " << std::endl;
		std::cout << "X : " << accl_stamped.data.x << std::endl;
		std::cout << "Y : " << accl_stamped.data.y << std::endl;
		std::cout << "Z : " << accl_stamped.data.z << std::endl;
		std::cout << accl_stamped.timestamp << std::endl;
	}

	bool result = imu.self_test();
	std::cout << result << std::endl;
}
