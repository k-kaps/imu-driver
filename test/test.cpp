#include <mpu6050_driver.hpp>

int main() {
    MPU6050Driver imu;
    imu.initializeDriver();
    imu.readAccelerometerValues();
    imu.readGyroscopeValues();
}
