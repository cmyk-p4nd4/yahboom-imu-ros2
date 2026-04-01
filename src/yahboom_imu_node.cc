
#include "yahboom_imu_ros2/yahboom_driver.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>

using namespace std::chrono_literals;

class IMUNode : public rclcpp::Node {
 public:
  IMUNode() : Node("yahboom_imu") {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 921600);
    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    sensor_ = std::make_unique<yb::IMU>(port, baudrate);
    float rate = this->getLookupRate(sensor_->getOutputRate());

    RCLCPP_INFO(this->get_logger(), "IMU Output Rate is %.f", rate);

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::QoS(20));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS(20));

    imu_timer_ = this->create_timer(std::chrono::duration<double, std::milli>(1000.f / rate), std::bind(&IMUNode::publish_imu_data, this));
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
    auto odom_message = nav_msgs::msg::Odometry();
    imu_message.header.stamp = this->get_clock()->now();
    imu_message.header.frame_id = "imu_link";

    odom_message.header.stamp = this->get_clock()->now();
    odom_message.header.frame_id = "imu_odom";
    odom_message.child_frame_id = "imu_link";

    // Fetch data
    auto accel = sensor_->getAcceleration();
    auto gyro = sensor_->getAngularVelocity();
    auto quat = sensor_->getQuaternion();
    auto angle = sensor_->getRollPitchYaw();

    // Populate
    imu_message.linear_acceleration.x = accel.getX();
    imu_message.linear_acceleration.y = accel.getY();
    imu_message.linear_acceleration.z = accel.getZ();

    imu_message.angular_velocity.x = gyro.getX();
    imu_message.angular_velocity.y = gyro.getY();
    imu_message.angular_velocity.z = gyro.getZ();

    imu_message.orientation.x = quat.getX();
    imu_message.orientation.y = quat.getY();
    imu_message.orientation.z = quat.getZ();
    imu_message.orientation.w = quat.getW();

    odom_message.twist.twist.angular.x = gyro.getX();
    odom_message.twist.twist.angular.y = gyro.getY();
    odom_message.twist.twist.angular.z = gyro.getZ();

    odom_message.twist.twist.linear.x = accel.getX();
    odom_message.twist.twist.linear.y = accel.getY();
    odom_message.twist.twist.linear.z = accel.getZ();

    odom_message.pose.pose.orientation.x = quat.getX();
    odom_message.pose.pose.orientation.y = quat.getY();
    odom_message.pose.pose.orientation.z = quat.getZ();
    odom_message.pose.pose.orientation.w = quat.getW();

    // Publish the message
    imu_publisher_->publish(imu_message);
    odom_publisher_->publish(odom_message);
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::TimerBase::SharedPtr imu_timer_;

  std::unique_ptr<yb::IMU> sensor_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();

  return 0;
}