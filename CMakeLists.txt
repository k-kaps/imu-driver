cmake_minimum_required(VERSION 2.8)
project(imu-driver)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE_DIR}/src/mpu6050_driver.cpp
    ${PROJECT_SOURCE_DIR}/include/mpu6050_driver.hpp
)

add_executable(test 
    ${PROJECT_SOURCE_DIR}/test/test.cpp
)

target_link_libraries(${PROJECT_NAME}
	i2c
)

target_link_libraries(test
    ${PROJECT_NAME}
)
