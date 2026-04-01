#pragma once

#include <cstdint>
#include <type_traits>

namespace yb {

#define YB_EXPORT __attribute__((visibility("default")))

enum class YB_EXPORT IMURegister : uint8_t {
  // Control and Configuration Registers (R/W)
  SAVE = 0x00,         // Save settings / Reboot / Factory reset
  CALSW = 0x01,        // Calibration mode (Auto, Height reset, Heading reset, etc.)
  RSW = 0x02,          // Output content selection (Select which data to broadcast)
  RRATE = 0x03,        // Data output rate (0.2Hz to 200Hz)
  BAUD = 0x04,         // Serial port baud rate (4800 to 921600 bps)
  AX_OFFSET = 0x05,    // Acceleration X Bias
  AY_OFFSET = 0x06,    // Acceleration Y Bias
  AZ_OFFSET = 0x07,    // Acceleration Z Bias
  GX_OFFSET = 0x08,    // Angular velocity X Bias
  GY_OFFSET = 0x09,    // Angular velocity Y Bias
  GZ_OFFSET = 0x0A,    // Angular velocity Z Bias
  HX_OFFSET = 0x0B,    // Magnetic Field X Bias
  HY_OFFSET = 0x0C,    // Magnetic Field Y Bias
  HZ_OFFSET = 0x0D,    // Magnetic Field Z Bias
  D0_MODE = 0x0E,      // D0 Pin mode (Digital, Analog, PWM, etc.)
  D1_MODE = 0x0F,      // D1 Pin mode (Digital, Analog, PWM, etc.)
  D2_MODE = 0x10,      // D2 Pin mode (Digital, Analog, PWM, etc.)
  D3_MODE = 0x11,      // D3 Pin mode (Digital, Analog, PWM, etc.)
  I2CADDR = 0x1A,      // I2C device address
  LEDOFF = 0x1B,       // Turn off/on the LED lights
  MAG_RANGE_X = 0x1C,  // Magnetic Field X Calibration Range
  MAG_RANGE_Y = 0x1D,  // Magnetic Field Y Calibration Range
  MAG_RANGE_Z = 0x1E,  // Magnetic Field Z Calibration Range
  BANDWIDTH = 0x1F,    // Sensor bandwidth (5Hz to 256Hz)
  GYRORANGE = 0x20,    // Gyroscope range settings
  ACCRANGE = 0x21,     // Accelerometer range settings
  SLEEP = 0x22,        // Hibernate/Wake up
  ORIENT = 0x23,       // Installation direction (Horizontal/Vertical)
  AXIS6 = 0x24,        // Algorithm selection (6-axis or 9-axis)
  FILTK = 0x25,        // Dynamic filtering K value
  GPSBAUD = 0x26,      // GPS module baud rate
  READADDR = 0x27,     // Address to read for register polling
  ACCFILT = 0x2A,      // Acceleration filter parameters
  POWONSEND = 0x2D,    // Command to start on power-on
  VERSION = 0x2E,      // Module version number
  YYMM = 0x30,         // Date: Year and Month
  DDHH = 0x31,         // Date: Day and Hour
  MMSS = 0x32,         // Time: Minute and Second
  MS = 0x33,           // Time: Millisecond
  ACC_X = 0x34,        // Acceleration X-axis
  ACC_Y = 0x35,        // Acceleration Y-axis
  ACC_Z = 0x36,        // Acceleration Z-axis
  GYRO_X = 0x37,       // Angular velocity X-axis
  GYRO_Y = 0x38,       // Angular velocity Y-axis
  GYRO_Z = 0x39,       // Angular velocity Z-axis
  MAG_X = 0x3A,        // Magnetic field X-axis
  MAG_Y = 0x3B,        // Magnetic field Y-axis
  MAG_Z = 0x3C,        // Magnetic field Z-axis
  ROLL = 0x3D,         // Euler Angle: Roll
  PITCH = 0x3E,        // Euler Angle: Pitch
  YAW = 0x3F,          // Euler Angle: Yaw (Heading)
  TEMP = 0x40,         // Temperature
  DIGITAL_0 = 0x41,    // D0 pin state/voltage
  DIGITAL_1 = 0x42,    // D1 pin state/voltage
  DIGITAL_2 = 0x43,    // D2 pin state/voltage
  DIGITAL_3 = 0x44,    // D3 pin state/voltage
  PRESSURE_L = 0x45,   // Air pressure (Low 16 bits)
  PRESSURE_H = 0x46,   // Air pressure (High 16 bits)
  HEIGHT_L = 0x47,     // Height (Low 16 bits)
  HEIGHT_H = 0x48,     // Height (High 16 bits)
  LON_L = 0x49,        // Longitude (Low 16 bits)
  LON_H = 0x4A,        // Longitude (High 16 bits)
  LAT_L = 0x4B,        // Latitude (Low 16 bits)
  LAT_H = 0x4C,        // Latitude (High 16 bits)
  GPS_HEIGHT = 0x4D,   // GPS Altitude
  GPS_YAW = 0x4E,      // GPS Heading angle
  GPS_VL = 0x4F,       // GPS Ground speed (Low 16 bits)
  GPS_VH = 0x50,       // GPS Ground speed (High 16 bits)
  QUAT_0 = 0x51,       // Quaternion q0
  QUAT_1 = 0x52,       // Quaternion q1
  QUAT_2 = 0x53,       // Quaternion q2
  QUAT_3 = 0x54,       // Quaternion q3
  SVNUM = 0x55,        // Number of GPS satellites
  PDOP = 0x56,         // GPS Position accuracy
  HDOP = 0x57,         // GPS Horizontal accuracy
  VDOP = 0x58,         // GPS Vertical accuracy
  XMIN = 0x5A,         // X-axis angle alarm minimum
  XMAX = 0x5B,         // X-axis angle alarm maximum
  BATVAL = 0x5C,       // Supply voltage (Read only)
  ALARMPIN = 0x5D,     // Alarm Pin Mapping
  YMIN = 0x5E,         // Y-axis angle alarm minimum
  YMAX = 0x5F,         // Y-axis angle alarm maximum
  GYROCALITHR = 0x61,  // Gyro Still Threshold
  ALARMLEVEL = 0x62,   // Angle alarm level
  GYROCALTIME = 0x63,  // Gyroscope Auto Calibration Time
  TRIGTIME = 0x68,     // Alarm continuous trigger time
  KEY = 0x69,          // Unlock command register
  WERROR = 0x6A,       // Gyroscope change value (Read only)
  TIMEZONE = 0x6B,     // GPS Time zone
  WZTIME = 0x6E,       // Angular velocity continuous still time
  WZSTATIC = 0x6F,     // Angular velocity integral threshold
  MODDELAY = 0x74,     // 485 data response delay
  XREFROLL = 0x79,     // Roll angle zero reference value (Read only)
  YREFPITCH = 0x7A,    // Pitch angle zero reference value (Read only)

  // Device Identification (Read Only)
  NUMBER_ID1 = 0x7F,  // Device ID 1-2
  NUMBER_ID2 = 0x80,  // Device ID 3-4
  NUMBER_ID3 = 0x81,  // Device ID 5-6
  NUMBER_ID4 = 0x82,  // Device ID 7-8
  NUMBER_ID5 = 0x83,  // Device ID 9-10
  NUMBER_ID6 = 0x84   // Device ID 11-12
};

enum class YB_EXPORT IMUPacketType : uint8_t {
  TIME = 0x50,              // Time output
  ACCELERATION = 0x51,      // Acceleration output
  ANGULAR_VELOCITY = 0x52,  // Angular velocity output
  ANGLE = 0x53,             // Angle output
  MAGNETIC_FIELD = 0x54,    // Magnetic field output
  PORT_STATUS = 0x55,       // Port status output
  BAROMETRIC_ALT = 0x56,    // barometric output
  LAT_LON = 0x57,           // Latitude and longitude output
  GROUND_SPEED = 0x58,      // GPS Ground speed output
  QUATERNION = 0x59,        // Quaternion output
  GPS_ACCURACY = 0x5A,      // GPS positioning accuracy output
  READ_REGISTER = 0x5F,     // Read value from register
};

enum class YB_EXPORT IMUAlgorithm : uint8_t {
  ALGO_AXIS9 = 0x00,  // 9-axis algorithm (magnetic field solution navigation angle, absolute heading angle)
  ALGO_AXIS6 = 0x01,  // 6-axis algorithm (integral solution navigation angle, relative heading angle)
};

enum class YB_EXPORT IMUOutputRate : uint8_t {
  RATE_0_2_HZ = 0x01,
  RATE_0_5_HZ = 0x02,
  RATE_1_HZ = 0x03,
  RATE_2_HZ = 0x04,
  RATE_5_HZ = 0x05,
  RATE_10_HZ = 0x06,
  RATE_20_HZ = 0x07,
  RATE_50_HZ = 0x08,
  RATE_100_HZ = 0x09,
  RATE_200_HZ = 0x0B,
  RATE_ONESHOT = 0x0C,
  RATE_NO_RETURN = 0x0D
};

enum class YB_EXPORT IMUCalibrationMode : uint8_t {
  CALIB_NORMAL = 0x00,          // normal operation mode
  CALIB_AUTOSUM = 0x01,         // auto sum-up calibration
  CALIB_HEIGHT_ZERO = 0x03,     // height reset
  CALIB_ANGLE = 0x04,           // heading angle reset
  CALIB_MAG_SPHERE_FIT = 0x07,  // Magnetic Field Calibration by Spherical Fitting
  CALIB_ANGLE_REF = 0x08,       // Set angle reference
  CALIB_MAG_DUAL_PLANE = 0x09,  // Magnetic Field Calibration by Dual Plane
};

enum class YB_EXPORT IMURSWFlag : uint16_t {
  RSW_TIME = 0x0001,              // Timestamp output
  RSW_ACCELERATION = 0x0002,      // Acceleration output
  RSW_ANGULAR_VELOCITY = 0x0004,  // Angular velocity output
  RSW_ANGLE = 0x0008,             // Angle output
  RSW_MAGNETIC_FIELD = 0x0010,    // Magnetic field output
  RSW_PORT_STATUS = 0x0020,       // Port status output
  RSW_BAROMETRIC_ALT = 0x0040,    // barometric output
  RSW_LAT_LON = 0x0080,           // Latitude and longitude output
  RSW_GROUND_SPEED = 0x0100,      // GPS Ground speed output
  RSW_QUATERNION = 0x0200,        // Quaternion output
  RSW_GPS_ACCURACY = 0x0400,      // GPS positioning accuracy output
};

YB_EXPORT
constexpr inline IMURSWFlag operator|(IMURSWFlag lhs, IMURSWFlag rhs) {
  using T = std::underlying_type_t<IMURSWFlag>;
  return static_cast<IMURSWFlag>(static_cast<T>(lhs) | static_cast<T>(rhs));
}

YB_EXPORT
constexpr inline IMURSWFlag operator&(IMURSWFlag lhs, IMURSWFlag rhs) {
  using T = std::underlying_type_t<IMURSWFlag>;
  return static_cast<IMURSWFlag>(static_cast<T>(lhs) & static_cast<T>(rhs));
}

YB_EXPORT
constexpr inline IMURSWFlag operator^(IMURSWFlag lhs, IMURSWFlag rhs) {
  using T = std::underlying_type_t<IMURSWFlag>;
  return static_cast<IMURSWFlag>(static_cast<T>(lhs) ^ static_cast<T>(rhs));
}

YB_EXPORT
constexpr inline IMURSWFlag operator~(IMURSWFlag val) {
  using T = std::underlying_type_t<IMURSWFlag>;
  return static_cast<IMURSWFlag>(~static_cast<T>(val));
}

YB_EXPORT
inline IMURSWFlag& operator|=(IMURSWFlag& lhs, IMURSWFlag rhs) {
  lhs = lhs | rhs;
  return lhs;
}

YB_EXPORT
inline IMURSWFlag& operator&=(IMURSWFlag& lhs, IMURSWFlag rhs) {
  lhs = lhs & rhs;
  return lhs;
}

YB_EXPORT
inline IMURSWFlag& operator^=(IMURSWFlag& lhs, IMURSWFlag rhs) {
  lhs = lhs ^ rhs;
  return lhs;
}

}  // namespace yb
