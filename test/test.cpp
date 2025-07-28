#include <mpu6050_driver.hpp>

bool test_read_accl() {
	MPU6050Driver imu = MPU6050Driver();
	
	imu.init_driver();
	
	AcclStamped accl_stamped;
	if(imu.read_accl(accl_stamped)) {
		std::cout << "Timestamp : " << accl_stamped.timestamp << std::endl;

		std::cout << "Accelerometer Data : " << std::endl;
		std::cout << "X : " << accl_stamped.data.x << std::endl;
		std::cout << "Y : " << accl_stamped.data.y << std::endl;
		std::cout << "Z : " << accl_stamped.data.z << std::endl;
		
		return true;
	}
	return false;
}

bool test_read_gyro() {
	MPU6050Driver imu = MPU6050Driver();
	
	imu.init_driver();
	
	AcclStamped gyro_stamped;
	if(imu.read_accl(gyro_stamped)) {
		std::cout << "Timestamp : " << gyro_stamped.timestamp << std::endl;

		std::cout << "Accelerometer Data : " << std::endl;
		std::cout << "X : " << gyro_stamped.data.x << std::endl;
		std::cout << "Y : " << gyro_stamped.data.y << std::endl;
		std::cout << "Z : " << gyro_stamped.data.z << std::endl;

		return true;
	}
	return false;
}

bool test_read_imu() {
	MPU6050Driver imu = MPU6050Driver();
	
	imu.init_driver();

	IMUStamped imu_stamped;
	if(imu.read_imu(imu_stamped)) {
		std::cout << "Timestamp : " << imu_stamped.timestamp << std::endl;

		std::cout << "Accelerometer Data : " << std::endl;
		std::cout << "X : " << imu_stamped.data.accl.x << std::endl;
		std::cout << "Y : " << imu_stamped.data.accl.y << std::endl;
		std::cout << "Z : " << imu_stamped.data.accl.z << std::endl;

		std::cout << "Gyroscope Data : " << std::endl;
		std::cout << "X : " << imu_stamped.data.gyro.x << std::endl;
		std::cout << "Y : " << imu_stamped.data.gyro.y << std::endl;
		std::cout << "Z : " << imu_stamped.data.gyro.z << std::endl;

		return true;
	}
	return false;
}

int main() {
	std::cout << "[TEST 1] : READ ACCELEROMETER DATA" << std::endl;
	std::string result1 = test_read_accl() ? "PASSED" : "FAILED";
	std::cout << "[TEST 1] : " << result1 << std::endl;

	std::cout << "[TEST 2] : READ GYROSCOPE DATA" << std::endl;
	std::string result2 = test_read_gyro() ? "PASSED" : "FAILED";
	std::cout << "[TEST 2] : " << result2 << std::endl;
	
	std::cout << "[TEST 3] : READ IMU DATA" << std::endl;
	std::string result3 = test_read_accl() ? "PASSED" : "FAILED";
	std::cout << "[TEST 3] : " << result3 << std::endl;
}
