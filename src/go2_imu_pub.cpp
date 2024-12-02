#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

using std::placeholders::_1;

class IMUPublisher : public rclcpp::Node
{
public:
  IMUPublisher() : Node("imu_publisher")
  {
    // Create a publisher to publish IMU data
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

    // Create a subscription to receive sport mode state data
    sport_mode_state_subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "sportmodestate", 10, std::bind(&IMUPublisher::sportModeStateCallback, this, _1));
  }

private:
  void sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr sport_mode_state_msg)
  {
    // Extract IMU data from the received sport mode state message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now(); // Set current ROS time as timestamp

    // Convert quaternion to orientation
    imu_msg.orientation.x = sport_mode_state_msg->imu_state.quaternion[0];
    imu_msg.orientation.y = sport_mode_state_msg->imu_state.quaternion[1];
    imu_msg.orientation.z = sport_mode_state_msg->imu_state.quaternion[2];
    imu_msg.orientation.w = sport_mode_state_msg->imu_state.quaternion[3];

    // Assign gyroscope data
    imu_msg.angular_velocity.x = sport_mode_state_msg->imu_state.gyroscope[0];
    imu_msg.angular_velocity.y = sport_mode_state_msg->imu_state.gyroscope[1];
    imu_msg.angular_velocity.z = sport_mode_state_msg->imu_state.gyroscope[2];

    // Assign accelerometer data
    imu_msg.linear_acceleration.x = sport_mode_state_msg->imu_state.accelerometer[0];
    imu_msg.linear_acceleration.y = sport_mode_state_msg->imu_state.accelerometer[1];
    imu_msg.linear_acceleration.z = sport_mode_state_msg->imu_state.accelerometer[2];

    // Publish the IMU data
    imu_publisher_->publish(imu_msg);
    printf("call_back");
  }

  // Publisher for IMU data
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  // Subscription for sport mode state data
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_mode_state_subscription_;
};

int main(int argc, char *argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create an instance of the IMUPublisher node
  auto node = std::make_shared<IMUPublisher>();

  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}

