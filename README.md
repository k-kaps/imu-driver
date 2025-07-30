# IMU Driver

A user-space driver for the MPU-6050 IMU for Linux.

A ROS 2 package to use this driver and publish the IMU data as sensor messages has also been created [here](https://github.com/k-kaps/ros2-imu-driver)

### Project Structure
This project has been structured as follows.
```
imu-driver
├── include
│   ├── helpers.hpp
│   └── mpu6050_driver.hpp
├── src
│   └── mpu6050_driver.cpp
├── test
│   └── test.cp
├── CMakeLists.txt
└── README.md
```
The `mpu6050_driver.hpp` file contains the `MPU6050Driver` class and its member functions. All the helper macros, structs, and enums have been defined in the `helpers.hpp` file.

### Build
This project builds a library for the IMU and a small test executable which tests basic functionality of capturing accelerometer and gyroscope values, and testing the IMU's FIFO buffer. The project can be built by:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

### Resources
- [MPU-6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU-6050 Register Maps](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)