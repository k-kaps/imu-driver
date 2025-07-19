#include <mpu6050_driver.hpp>

MPU6050Driver::MPU6050Driver(int x, int y) {
	driver_val = x + y;
}

int main() {
	std::cout << "Hello, World!" << std::endl;
}
