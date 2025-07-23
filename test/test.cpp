#include <mpu6050_driver.hpp>

int main() {
    MPU6050Driver imu;
    imu.init_driver();
    imu.read_accl();
}
