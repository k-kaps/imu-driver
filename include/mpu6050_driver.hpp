extern "C" {
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}
#include <iostream>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <sys/ioctl.h>
#include <unistd.h>
#include <unordered_map>
#include <helpers.hpp>

class MPU6050Driver {
public:
	MPU6050Driver();
	MPU6050Driver(MPU6050Config& config);
	~MPU6050Driver();
	
	void init_driver();

    bool read_accl(AcclStamped& accl_stamped);
	bool read_accl(AcclData& accl_data);

	bool read_gyro(GyroStamped& gyro_stamped);
	bool read_gyro(GyroData& gyro_data);

	bool read_imu(IMUStamped& imu_stamped);
	bool read_imu(IMUData& imu_data);

	bool read_temp(TempStamped& temp_stamped);

	void set_fifo_enabled(bool enable, FifoCFG conf);
	bool get_fifo_status();
	bool read_fifo_buffer(uint8_t* buf, uint16_t size);
	bool check_fifo_count(uint16_t size);

private:
	int file_;
	bool init_;
	MPU6050Config config_;
	IMUData imu_ft_;
	std::unordered_map<int, float> accl_map_ = 
		{{AcclFSR::A_FSR_2G, 16384.0}, {AcclFSR::A_FSR_4G, 8192.0}, 
		{AcclFSR::A_FSR_8G, 4096.0}, {AcclFSR::A_FSR_16G, 2048.0}};
	std::unordered_map<int, float> gyro_map_ = 
		{{GyroFSR::G_FSR_250, 131.0}, {GyroFSR::G_FSR_500, 65.5}, 
		{GyroFSR::G_FSR_1000, 32.8}, {GyroFSR::G_FSR_2000, 16.4}};

	void unpack_hl_vals(uint8_t* hl_buf, int16_t* raw_buf);
	int16_t combine_hl_vals(uint8_t* buf);
	void process_raw_buf(AcclData& accl_data, int16_t* raw_buf);
	void process_raw_buf(GyroData& gyro_data, int16_t* raw_buf);
	double process_raw_accl_val(int16_t accl_val);
	double process_raw_gyro_val(int16_t gyro_val);
	double process_raw_temp_val(int16_t temp_val);

	void configure_imu();
	void configure_fifo(FifoCFG conf);
	void reset_fifo_buffer();

	bool get_ft_vals();
	void ft_get_raw_vals(uint8_t* accl_ft, uint8_t* gyro_ft);
	void compute_accl_ft(uint8_t* accl_ft);
	void compute_gyro_ft(uint8_t* gyro_ft);
	
	void read_reg(char reg_addr, uint8_t* out_buf, uint8_t size);

	void error_handler(const std::string& message);
	bool check_init();
};
