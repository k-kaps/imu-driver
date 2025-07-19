cmake_minimum_required(VERSION 3.22)
project(imu_driver)

add_executable(mpu6050_driver src/mpu6050_driver.cpp)

target_include_directories(mpu6050_driver PRIVATE include)

