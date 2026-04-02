#include "yahboom_imu_ros2/yahboom_math.h"
#include "yahboom_imu_ros2/yahboom_typedefs.h"
#include "yahboom_imu_ros2/yahboom_driver.h"

//
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <system_error>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>

namespace yb {

constexpr inline bool HasFlag(IMURSWFlag value, IMURSWFlag flag) {
  return (value & flag) == flag;
}

static std::string getString(std::vector<uint8_t> data);

IMU::IMU()
  : acceleration_(),
    angular_speed_(),
    magnetic_field_(),
    rpy_angle_(),
    quat_(),
    temperature_(),
    ser_(nullptr) {
}

IMU::IMU(const std::string& port_name, int baudrate)
  : IMU() {
  this->attachDevice(port_name, baudrate);
}

IMU::~IMU() {
  if (ser_ != nullptr) {
    this->detachDevice();
  }
}

void IMU::attachDevice(const std::string& port_name, int baudrate) {
  ser_ = new Serial(port_name, baudrate);
  ser_->subscribeOnRead(std::bind(&IMU::handleIMUMessage, this, std::placeholders::_1));
  ser_->begin();

  this->fetchConfigs();
}

void IMU::detachDevice(void) {
  ser_->end();
  ser_->closePort();
  delete ser_;
  ser_ = nullptr;
}

bool IMU::isConnected(void) const {
  if (ser_ == nullptr) {
    return false;
  }
  return ser_->opened();
}

Vector3 IMU::getAcceleration(void) {
  if (HasFlag(rsw_status_, IMURSWFlag::RSW_ACCELERATION)) {
    return acceleration_;
  } else {
    ScaledVector3 s_accl;
    Vector3 accl;
    const float factor = 16.f * 9.8065f / 32768.f;
    ResponseData data = this->readRegister(IMURegister::ACC_X);
    s_accl.getX() = data.at(0);
    s_accl.getY() = data.at(1);
    s_accl.getZ() = data.at(2);
    convertScaledVector(&s_accl, factor, &accl);
    return accl;
  }
}

Vector3 IMU::getAngularVelocity(void) {
  if (HasFlag(rsw_status_, IMURSWFlag::RSW_ANGULAR_VELOCITY)) {
    return angular_speed_;
  } else {
    ScaledVector3 s_gyro;
    Vector3 gyro;
    const float factor = 2000.f / 32768.f;
    ResponseData data = this->readRegister(IMURegister::GYRO_X);
    s_gyro.getX() = data.at(0);
    s_gyro.getY() = data.at(1);
    s_gyro.getZ() = data.at(2);
    convertScaledVector(&s_gyro, factor, &gyro);
    return gyro;
  }
}

Vector3 IMU::getMagneticField(void) {
  if (HasFlag(rsw_status_, IMURSWFlag::RSW_MAGNETIC_FIELD)) {
    return magnetic_field_;
  } else {
    ScaledVector3 s_magnetic;
    Vector3 magnetic;
    const float factor = 1.0f;
    ResponseData data = this->readRegister(IMURegister::MAG_X);
    s_magnetic.getX() = data.at(0);
    s_magnetic.getY() = data.at(1);
    s_magnetic.getZ() = data.at(2);
    convertScaledVector(&s_magnetic, factor, &magnetic);
    return magnetic;
  }
}

Vector3 IMU::getRollPitchYaw(void) {
  if (HasFlag(rsw_status_, IMURSWFlag::RSW_ANGLE)) {
    return rpy_angle_;
  } else {
    ScaledVector3 s_xyz_angle;
    Vector3 xyz_angle;
    const float factor = 180.f / 32768.f;
    ResponseData data = this->readRegister(IMURegister::ROLL);
    s_xyz_angle.getX() = data.at(0);
    s_xyz_angle.getY() = data.at(1);
    s_xyz_angle.getZ() = data.at(2);
    convertScaledVector(&s_xyz_angle, factor, &xyz_angle);
    return xyz_angle;
  }
}

Quaternion IMU::getQuaternion(void) {
  if (HasFlag(rsw_status_, IMURSWFlag::RSW_QUATERNION)) {
    return quat_;
  } else {
    ScaledQuaternion s_quat;
    Quaternion quat;
    const float factor = 1.f / 32768.f;
    ResponseData data = this->readRegister(IMURegister::QUAT_0);
    s_quat.getW() = data.at(0);
    s_quat.getX() = data.at(1);
    s_quat.getY() = data.at(2);
    s_quat.getZ() = data.at(3);
    convertScaledQuaternion(&s_quat, factor, &quat);
    return quat;
  }
}

float IMU::getTemperature(void) {
  if (HasFlag(rsw_status_, IMURSWFlag::RSW_ACCELERATION) || HasFlag(rsw_status_, IMURSWFlag::RSW_MAGNETIC_FIELD) || HasFlag(rsw_status_, IMURSWFlag::RSW_ANGLE)) {
    return temperature_;
  } else {
    ResponseData data = this->readRegister(IMURegister::TEMP);
    float celsius = static_cast<float>(data.at(0)) / 100.f;
    return celsius;
  }
}

Vector3 IMU::getBarometricAltitude(void) {
  if (HasFlag(rsw_status_, IMURSWFlag::RSW_BAROMETRIC_ALT)) {
    return Vector3((float)(barometric_pressure_) / 1000.f, (float)(barometric_height_) / 10.f, 0);
  } else {
    Vector3 baro_alt;
    ResponseData data = this->readRegister(IMURegister::PRESSURE_L);
    baro_alt.getX() = (float)(static_cast<int32_t>(data.at(0)) | static_cast<int32_t>(data.at(1)) << 16) / 1000.f;
    baro_alt.getY() = (float)(static_cast<int32_t>(data.at(2)) | static_cast<int32_t>(data.at(3)) << 16) / 10.f;
    return baro_alt;
  }
}

Vector3 IMU::getLatitudeLongitude(void) {
  throw std::system_error(std::make_error_code(std::errc::function_not_supported), "Function not supported");
}

Vector3 IMU::getGPSGroundSpeed(void) {
  throw std::system_error(std::make_error_code(std::errc::function_not_supported), "Function not supported");
}

uint16_t IMU::getGPSSatelliteCount(void) {
  throw std::system_error(std::make_error_code(std::errc::function_not_supported), "Function not supported");
}

void IMU::saveSettings(void) {
  this->writeRegister(IMURegister::SAVE, 0x0000);
}

void IMU::factoryReset(void) {
  this->writeRegister(IMURegister::SAVE, 0x0001);
}

void IMU::setCalibrationMode(IMUCalibrationMode mode) {
  this->writeRegister(IMURegister::CALSW, static_cast<uint16_t>(mode));
}

void IMU::setVerticalPlacement(bool enable) {
  this->writeRegister(IMURegister::ORIENT, static_cast<uint16_t>(enable));
}

void IMU::setAlgorithm(IMUAlgorithm algo) {
  this->writeRegister(IMURegister::AXIS6, static_cast<uint16_t>(algo));
}

void IMU::setAccelerationBias(const Vector3& offset, const uint8_t mask) {
  ScaledVector3 s_offset;
  convertUnscaledVector(&offset, 10000.f, &s_offset);

  if ((mask & 0x01) == 0x01) {
    this->writeRegister(IMURegister::AX_OFFSET, s_offset.getX());
  }

  if ((mask & 0x02) == 0x02) {
    this->writeRegister(IMURegister::AY_OFFSET, s_offset.getY());
  }

  if ((mask & 0x04) == 0x04) {
    this->writeRegister(IMURegister::AZ_OFFSET, s_offset.getZ());
  }
}

Vector3 IMU::getAccelerationBias(void) {
  ScaledVector3 s_offset;
  Vector3 offset;
  ResponseData data = this->readRegister(IMURegister::AX_OFFSET);
  s_offset.getX() = data.at(0);
  s_offset.getY() = data.at(1);
  s_offset.getZ() = data.at(2);
  convertScaledVector(&s_offset, 1.0f / 10000.0f, &offset);
  return offset;
}

void IMU::setAngularVelocityBias(const Vector3& offset, const uint8_t mask) {
  ScaledVector3 s_offset;
  convertUnscaledVector(&offset, 10000.f, &s_offset);

  if ((mask & 0x01) == 0x01) {
    this->writeRegister(IMURegister::GX_OFFSET, s_offset.getX());
  }

  if ((mask & 0x02) == 0x02) {
    this->writeRegister(IMURegister::GY_OFFSET, s_offset.getY());
  }

  if ((mask & 0x04) == 0x04) {
    this->writeRegister(IMURegister::GZ_OFFSET, s_offset.getZ());
  }
}

Vector3 IMU::getAngularVelocityBias(void) {
  ScaledVector3 s_offset;
  Vector3 offset;
  ResponseData data = this->readRegister(IMURegister::GX_OFFSET);
  s_offset.getX() = data.at(0);
  s_offset.getY() = data.at(1);
  s_offset.getZ() = data.at(2);
  convertScaledVector(&s_offset, 1.0f / 10000.0f, &offset);
  return offset;
}

void IMU::setMagneticFieldBias(const Vector3& offset, const uint8_t mask) {
  ScaledVector3 s_offset;
  convertUnscaledVector(&offset, 1.0f, &s_offset);

  if ((mask & 0x01) == 0x01) {
    this->writeRegister(IMURegister::HX_OFFSET, s_offset.getX());
  }

  if ((mask & 0x02) == 0x02) {
    this->writeRegister(IMURegister::HY_OFFSET, s_offset.getY());
  }

  if ((mask & 0x04) == 0x04) {
    this->writeRegister(IMURegister::HZ_OFFSET, s_offset.getZ());
  }
}

Vector3 IMU::getMagneticFieldBias(void) {
  ScaledVector3 s_offset;
  Vector3 offset;
  ResponseData data = this->readRegister(IMURegister::HX_OFFSET);
  s_offset.getX() = data.at(0);
  s_offset.getY() = data.at(1);
  s_offset.getZ() = data.at(2);
  convertScaledVector(&s_offset, 1.0f, &offset);
  return offset;
}

void IMU::setXAxisAlarmThreshold(float min_angle, float max_angle) {
  throw NotImplementedError();
}

void IMU::setYAxisAlarmThreshold(float min_angle, float max_angle) {
  throw NotImplementedError();
}

void IMU::setGyroStaticThreshold(uint16_t threshold) {
  throw NotImplementedError();
}

void IMU::setAlarmDelay(uint16_t ms) {
}

IMURSWFlag IMU::getAutoOutputContent(void) {
  ResponseData value = this->readRegister(IMURegister::RSW);
  rsw_status_ = static_cast<IMURSWFlag>(value.at(0));

  return rsw_status_;
}

void IMU::setAutoOutputContent(IMURSWFlag flags) {
  this->configureRegister(IMURegister::RSW, static_cast<uint16_t>(flags));
  rsw_status_ = flags;
}

IMUOutputRate IMU::getOutputRate(void) {
  ResponseData data = this->readRegister(IMURegister::RRATE);
  rrate_ = static_cast<IMUOutputRate>(data.at(0) & 0xFF);
  return rrate_;
}

void IMU::setOutputRate(IMUOutputRate rate) {
  this->configureRegister(IMURegister::RRATE, static_cast<uint16_t>(rate));
  rrate_ = rate;
}

void IMU::setLEDOff(bool turnoff) {
  this->configureRegister(IMURegister::LEDOFF, static_cast<uint16_t>(turnoff));
}

uint16_t IMU::getVersion(void) {
  ResponseData data = this->readRegister(IMURegister::VERSION);
  return static_cast<uint16_t>(data.at(0));
}

uint16_t IMU::getBatteryVoltage(void) {
  throw std::system_error(std::make_error_code(std::errc::function_not_supported), "Function not supported");
}

std::string IMU::getDeviceID(void) {
  std::string device_id;
  device_id.reserve(12);
  ResponseData data = this->readRegister(IMURegister::NUMBER_ID1);
  for (size_t i = 0; i < data.size(); i++) {
    device_id.push_back(static_cast<char>(data.at(i) & 0xFF));
    device_id.push_back(static_cast<char>((data.at(i) >> 8) & 0xFF));
  }
  data = this->readRegister(IMURegister::NUMBER_ID5);
  for (size_t i = 0; i < 2; i++) {
    device_id.push_back(static_cast<char>(data.at(i) & 0xFF));
    device_id.push_back(static_cast<char>((data.at(i) >> 8) & 0xFF));
  }
  return device_id;
}

IMU::AwaitableResponse IMU::readRegisterAsync(const IMURegister& reg) {
  IMU::AwaitableResponse future;
  {
    std::lock_guard<std::mutex> lk(promise_mut_);
    pending_promise_.reset(new PendingResponse);
    future = pending_promise_->get_future();
  }
  try {
    // read request
    this->writeRegister(IMURegister::READADDR, (uint16_t)reg);
  } catch (...) {
    // the write may fail, throw an exception instead
    pending_promise_->set_exception(std::current_exception());
  }

  return future;
}

ResponseData IMU::readRegister(const IMURegister& reg) {
  auto fut = this->readRegisterAsync(reg);
  if (fut.valid()) {
    while (fut.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      std::printf("\x1b[1;31mFuture wait timeout\x1b[0m\n");
    }
  }
  return fut.get();
}

void IMU::configureRegister(const IMURegister& reg, const uint16_t& value) {
  this->unlock();
  this->writeRegister(reg, value);
  this->saveSettings();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::ignore = this->readRegister(reg);  // poll a read
}

//-------------- protected functions -------------------

void IMU::fetchConfigs(void) {
  std::printf("Fetching config\n");
  this->getAutoOutputContent();
  this->getOutputRate();
}

[[maybe_unused]] static std::string getString(std::vector<uint8_t> data) {
  std::stringstream ss;
  for (size_t i = 0; i < data.size(); ++i) {
    ss << std::hex << std::uppercase;
    ss << std::setfill('0') << std::setw(2);
    ss << (int)data.at(i) << " ";
  }
  ss << '\b';
  return ss.str();
}

void IMU::handleIMUMessage(const std::vector<uint8_t>& packet) {
  IMUPacketType type = (IMUPacketType)packet.at(1);
  ResponseData data;
  for (size_t i = 0; i < 4; i++) {
    data.at(i) = (int16_t)packet.at(2 + 2 * i) + (int16_t)(packet.at(2 + 2 * i + 1) << 8);
  }

  // std::printf("Type is: %02X\n", (int)type);
  // std::cout << std::quoted(__FUNCTION__) << ": " << getString(packet) << "\r\n";

  float factor = 1.f;

  switch (type) {
    case IMUPacketType::TIME:
      break;
    case IMUPacketType::ACCELERATION:
      factor = 16.f * 9.8065f / 32768.f;
      temperature_ = static_cast<float>(data.at(3)) / 100.f;
      for (size_t i = 0; i < 3; i++) {
        acceleration_.data[i] = static_cast<float>(data.at(i)) * factor;
      }
      break;
    case IMUPacketType::ANGULAR_VELOCITY:
      factor = 2000.f / 32768.f;
      temperature_ = static_cast<float>(data.at(3)) / 100.f;
      for (size_t i = 0; i < 3; i++) {
        angular_speed_.data[i] = static_cast<float>(data.at(i)) * factor;
      }
      break;
    case IMUPacketType::ANGLE:
      factor = 180.f / 32768.f;
      version_ = static_cast<uint16_t>(data.at(3));
      for (size_t i = 0; i < 3; i++) {
        rpy_angle_.data[i] = static_cast<float>(data.at(i)) * factor;
      }

      break;
    case IMUPacketType::MAGNETIC_FIELD:
      temperature_ = (float)data.at(3) / 100.f;
      for (size_t i = 0; i < 3; i++) {
        magnetic_field_.data[i] = static_cast<float>(data.at(i));
      }
      break;
    case IMUPacketType::PORT_STATUS:
      break;
    case IMUPacketType::BAROMETRIC_ALT:
      barometric_pressure_ = static_cast<int32_t>(data.at(0)) | static_cast<int32_t>(data.at(1)) << 16;
      barometric_height_ = static_cast<int32_t>(data.at(2)) | static_cast<int32_t>(data.at(3)) << 16;
      break;
    case IMUPacketType::LAT_LON:
      break;
    case IMUPacketType::GROUND_SPEED:
      break;
    case IMUPacketType::QUATERNION:
      factor = 1.f / 32768.f;
      for (size_t i = 0; i < 4; i++) {
        quat_.data[i] = (float)data.at(i) * factor;
      }
      break;
    case IMUPacketType::GPS_ACCURACY:
      break;
    case IMUPacketType::READ_REGISTER:
      // std::cout << getString(packet) << "\r\n";
      {
        std::scoped_lock<std::mutex> slk(promise_mut_);
        if (pending_promise_) {
          pending_promise_->set_value(std::move(data));
          pending_promise_.reset();
        }
      }
      break;
    default:
      break;
  }
}

void IMU::writeRegister(const IMURegister& reg, const uint16_t& value) {
  std::vector<uint8_t> data(5);
  data.at(0) = 0xFF;
  data.at(1) = 0xAA;
  data.at(2) = (uint8_t)(reg);
  data.at(3) = (uint8_t)(value & 0xFF);
  data.at(4) = (uint8_t)((value >> 8) & 0xFF);
  ser_->writeData(data);
}

void IMU::unlock(void) {
  this->writeRegister(IMURegister::KEY, 0xb588);
}

};  // namespace yb
