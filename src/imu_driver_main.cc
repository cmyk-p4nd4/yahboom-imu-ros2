#include "yahboom_imu_ros2/yahboom_driver.h"

#include <cstdio>
#include <csignal>
#include <chrono>
#include <thread>

volatile static bool g_exit = false;

static void sigint_handler(int signum) {
  g_exit = true;
}

constexpr inline static float getLookupRate(yb::IMUOutputRate rate) noexcept {
  switch (rate) {
    case yb::IMUOutputRate::RATE_0_2_HZ:
      return 0.2f;
    case yb::IMUOutputRate::RATE_0_5_HZ:
      return 0.5f;
    case yb::IMUOutputRate::RATE_1_HZ:
      return 1.f;
    case yb::IMUOutputRate::RATE_2_HZ:
      return 2.f;
    case yb::IMUOutputRate::RATE_5_HZ:
      return 5.f;
    case yb::IMUOutputRate::RATE_10_HZ:
      return 10.f;
    case yb::IMUOutputRate::RATE_20_HZ:
      return 20.f;
    case yb::IMUOutputRate::RATE_50_HZ:
      return 50.f;
    case yb::IMUOutputRate::RATE_100_HZ:
      return 100.f;
    case yb::IMUOutputRate::RATE_200_HZ:
      return 200.f;
    case yb::IMUOutputRate::RATE_ONESHOT:
      return -1.f;
    case yb::IMUOutputRate::RATE_NO_RETURN:
      return 0.f;
    default:
      return 0.f;
  }
}

int main(int argc, char** argv) {
  signal(SIGINT, sigint_handler);

  yb::IMU imu;
  imu.attachDevice("/dev/ttyUSB0", 921600);

  yb::IMUOutputRate imu_rate = imu.getOutputRate();
  yb::IMURSWFlag rsw_flag = imu.getAutoOutputContent();
  float rate = getLookupRate(imu_rate);
  std::printf("IMU Rate: %.1f\tOutput Fields: 0x%04X\n", rate, (uint16_t)rsw_flag);

  yb::IMUOutputRate new_rate{yb::IMUOutputRate::RATE_50_HZ};
  std::printf("Changing RRATE to %.1f\n", getLookupRate(new_rate));
  imu.setOutputRate(new_rate);

  imu_rate = imu.getOutputRate();
  rsw_flag = imu.getAutoOutputContent();
  rate = getLookupRate(imu_rate);
  std::printf("IMU Rate: %.1f\tOutput Fields: 0x%04X\n", rate, (uint16_t)rsw_flag);

  Vector3 accl, gyro, angle;
  Quaternion q;

  while (!g_exit && imu.isConnected()) {
    accl = imu.getAcceleration();
    gyro = imu.getAngularVelocity();
    angle = imu.getRollPitchYaw();
    q = imu.getQuaternion();

    std::printf("Accle: %+8.3f\t\t%+8.3f\t%+8.3f\n", accl.getX(), accl.getY(), accl.getZ());
    std::printf("Gyros: %+8.3f\t\t%+8.3f\t%+8.3f\n", gyro.getX(), gyro.getY(), gyro.getZ());
    std::printf("Angle: %+8.3f\t\t%+8.3f\t%+8.3f\n", angle.getX(), angle.getY(), angle.getZ());
    std::printf("Quat: %+8.3f\t\t%+8.3f\t%+8.3f\t%+8.3f\n", q.getX(), q.getY(), q.getZ(), q.getW());
    std::printf("\r\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(1000.0f / rate)));
  }

  if (!imu.isConnected()) {
    std::printf("IMU lost connection!\n");
  }
  imu.detachDevice();

  return 0;
}
