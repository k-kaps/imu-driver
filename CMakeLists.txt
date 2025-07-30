cmake_minimum_required(VERSION 2.8)
project(imu-driver)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE_DIR}/src/mpu6050_driver.cpp
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/mpu6050_driver.hpp
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

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Config
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
    DESTINATION ${CMAKE_PREFIX_PATH}/include
)

install(
    EXPORT ${PROJECT_NAME}Config
    FILE ${PROJECT_NAME}Config.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME}/
)