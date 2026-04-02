
#include "yahboom_imu_ros2/yahboom_driver.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>

using namespace std::chrono_literals;

class IMUNode : public rclcpp::Node {
public:
  IMUNode()
    : Node("yahboom_imu") {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 921600);
    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    sensor_ = std::make_unique<yb::IMU>(port, baudrate);
    float rate = this->getLookupRate(sensor_->getOutputRate());

    yb::IMURSWFlag rsw = sensor_->getAutoOutputContent();
    if ((rsw & yb::IMURSWFlag::RSW_MAGNETIC_FIELD) != yb::IMURSWFlag::RSW_MAGNETIC_FIELD) {
      sensor_->setAutoOutputContent(rsw | yb::IMURSWFlag::RSW_MAGNETIC_FIELD);
    }

    RCLCPP_INFO(this->get_logger(), "IMU Output Rate is %.f", rate);

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::QoS(20));
    mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("~/mag", rclcpp::QoS(20));
    // odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS(20));

    publisher_timer_ = this->create_timer(std::chrono::duration<double, std::milli>(1000.f / rate), std::bind(&IMUNode::publish_imu_data, this));
  }

  ~IMUNode() {
    if (sensor_) {
      sensor_->detachDevice();
    }
  }

protected:
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
        return 0.f;
      case yb::IMUOutputRate::RATE_NO_RETURN:
        return -1.f;
      default:
        return -1.f;
    }
  }

  void publish_imu_data() {
    auto imu_message = sensor_msgs::msg::Imu();
    imu_message.header.stamp = this->get_clock()->now();
    imu_message.header.frame_id = "imu_link";

    auto mag_message = sensor_msgs::msg::MagneticField();
    mag_message.header.stamp = this->get_clock()->now();
    mag_message.header.frame_id = "imu_link";

    // Fetch data
    auto accel = sensor_->getAcceleration();
    auto gyro = sensor_->getAngularVelocity();
    auto q = sensor_->getQuaternion();
    auto mag = sensor_->getMagneticField();

    // Populate
    imu_message.linear_acceleration.x = accel.getX();
    imu_message.linear_acceleration.y = accel.getY();
    imu_message.linear_acceleration.z = accel.getZ();

    imu_message.angular_velocity.x = gyro.getX() * M_PIf / 180.f;
    imu_message.angular_velocity.y = gyro.getY() * M_PIf / 180.f;
    imu_message.angular_velocity.z = gyro.getZ() * M_PIf / 180.f;

    imu_message.orientation.w = q.getW();
    imu_message.orientation.x = q.getX();
    imu_message.orientation.y = q.getY();
    imu_message.orientation.z = q.getZ();

    mag_message.magnetic_field.x = mag.getX();
    mag_message.magnetic_field.y = mag.getY();
    mag_message.magnetic_field.z = mag.getZ();

    // Publish the message
    imu_publisher_->publish(imu_message);
    mag_publisher_->publish(mag_message);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;

  std::unique_ptr<yb::IMU> sensor_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();

  return 0;
}