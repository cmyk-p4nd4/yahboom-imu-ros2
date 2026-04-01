#pragma once

#include "yahboom_imu_ros2/yahboom_typedefs.h"
#include "yahboom_imu_ros2/yahboom_math.h"
#include "yahboom_imu_ros2/serial.h"

#include <stdexcept>
#include <string>
#include <vector>
#include <array>
#include <future>

#define YB_EXPORT __attribute__((visibility("default")))

namespace yb {

using ResponseData = std::array<int16_t, 4>;

class YB_EXPORT NotImplementedError : public std::logic_error {
public:
  NotImplementedError()
    : std::logic_error("Function not yet implemented"){};
};

class YB_EXPORT IMU {
public:
  using AwaitableResponse = std::future<ResponseData>;
  using PendingResponse = std::promise<ResponseData>;

  IMU();
  IMU(const std::string& port_name, int baudrate);
  ~IMU();

  // --- Device Lifecycle ---
  void attachDevice(const std::string& port_name, int baudrate);
  void detachDevice(void);
  bool isConnected(void) const;

  Vector3 getAcceleration(void);
  Vector3 getAngularVelocity(void);
  Vector3 getMagneticField(void);
  Vector3 getRollPitchYaw(void);
  Quaternion getQuaternion(void);
  float getTemperature(void);

  // --- Data Acquisition (Environmental & GPS) ---
  Vector3 getBarometricAltitude(void);  // X: Air Pressure (kPa), Y: height (m), Z: 0
  Vector3 getLatitudeLongitude(void);   // X: Lon, Y: Lat, Z: GPS Height
  Vector3 getGPSGroundSpeed(void);      // Combined Ground Speed data
  uint16_t getGPSSatelliteCount(void);

  void saveSettings(void);  // Writes 0x0000 to SAVE register
  void factoryReset(void);  // Writes 0x0001 to SAVE register

  void setCalibrationMode(IMUCalibrationMode mode);
  // set the installation direction to vertical (Y-axis upward)
  void setVerticalPlacement(bool enable);
  void setAlgorithm(IMUAlgorithm algo);

  // --- Bias and Calibration Settings ---
  void setAccelerationBias(const Vector3& offset, const uint8_t mask = 0xF);
  Vector3 getAccelerationBias(void);
  void setAngularVelocityBias(const Vector3& offset, const uint8_t mask = 0xF);
  Vector3 getAngularVelocityBias(void);
  void setMagneticFieldBias(const Vector3& offset, const uint8_t mask = 0xF);
  Vector3 getMagneticFieldBias(void);

  // --- Alarms & Thresholds ---
  void setXAxisAlarmThreshold(float min_angle, float max_angle);
  void setYAxisAlarmThreshold(float min_angle, float max_angle);
  void setGyroStaticThreshold(uint16_t threshold);
  void setAlarmDelay(uint16_t ms);

  // --- Data Output Content ---
  IMURSWFlag getAutoOutputContent(void);        // Reads RSW register
  void setAutoOutputContent(IMURSWFlag flags);  // Writes RSW register
  IMUOutputRate getOutputRate(void);            // get how fast data is outputted
  void setOutputRate(IMUOutputRate rate);       // set how fast data is outputted

  // --- Device Status ---
  void setLEDOff(bool turnoff);
  uint16_t getVersion(void);
  uint16_t getBatteryVoltage(void);
  std::string getDeviceID(void);  // Reads NUMBERID1 through NUMBERID6

  // --- Register Access ---

  // \brief Read a Register. Returns a `std::future` that contains \n
  // the value of the register plus the 3 register after it
  // \param reg Register to read
  // \return Resolvable future that returns 4 register values
  // \note this method will not block. Implementation should wait for the promise
  // to resolve by calling `std::future::wait()` inorder to get the values
  AwaitableResponse readRegisterAsync(const IMURegister& reg);

  // \brief Read a Register. Returns the value of the register plus the 3 register after it
  // \param reg Register to read
  // \return an array of 4 register values
  // \details this method is just a blocking version of \c IMU::readRegisterAsync
  ResponseData readRegister(const IMURegister& reg);

  // \brief Configure a Register
  // \param[in] reg Register to configure
  // \param[in] value Value to configure
  void configureRegister(const IMURegister& reg, const uint16_t& value);

protected:
  // \brief Fetch configurations from the IMU
  void fetchConfigs(void);

  // \brief Callback function from read response
  void handleIMUMessage(const std::vector<uint8_t>& packet);

  // \brief Write 0xB588 to unlock the device
  void unlock(void);

  // \brief Write an unsigned 16 bit value to a register `reg`
  // \param[in] reg Register to write
  // \param[in] value value to write
  void writeRegister(const IMURegister& reg, const uint16_t& value);

  Vector3 acceleration_;
  Vector3 angular_speed_;
  Vector3 magnetic_field_;
  Vector3 rpy_angle_;
  Quaternion quat_;
  float temperature_;
  int32_t barometric_pressure_;
  int32_t barometric_height_;

  IMURSWFlag rsw_status_;
  IMUOutputRate rrate_;

  uint16_t version_;

  std::mutex promise_mut_;
  std::unique_ptr<PendingResponse> pending_promise_;

private:
  IMU(const IMU& other) = delete;
  IMU(IMU&& other) = delete;

  Serial* ser_ = nullptr;
};
}  // namespace yb
