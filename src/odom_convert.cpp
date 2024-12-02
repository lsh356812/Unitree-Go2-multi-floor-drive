#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("odom_publisher"), px(0), py(0), pz(0), roll(0), pitch(0), yaw(0), t(-1)
    {
        state_subscriber_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&OdomPublisher::state_callback, this, std::placeholders::_1));
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_convert", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&OdomPublisher::publish_odometry, this));
    }
private:
    void state_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        
            // Get initial position and orientation
            px = msg->position[0];
            py = msg->position[1];
            pz = msg->position[2];
            roll = msg->imu_state.rpy[0];
            pitch = msg->imu_state.rpy[1];
            yaw = msg->imu_state.rpy[2];
            RCLCPP_INFO(this->get_logger(), "Initial Pose: %f, %f, %f, %f, %f, %f", px, py, pz, roll, pitch, yaw);
            t = 0;  // Start time count
        
    }
    void publish_odometry()
    {
        if (t >= 0)
        {
            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header.stamp = this->get_clock()->now();
            odom_msg.header.frame_id = "odom_nav";
            odom_msg.child_frame_id = "base_footprint";
            // Set position
            odom_msg.pose.pose.position.x = px;
            odom_msg.pose.pose.position.y = py;
            odom_msg.pose.pose.position.z = pz;
            // Convert Euler angles to quaternion
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            odom_msg.pose.pose.orientation = tf2::toMsg(q);
            // Publish the message
            odom_publisher_->publish(odom_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing Odometry");
        }
    }
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double px, py, pz;  // Position coordinates
    double roll, pitch, yaw;  // Euler angles
    double t;  // Timer to start publishing after initial pose is set
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
// #include <cmath>
// #include <memory>
// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "unitree_go/msg/sport_mode_state.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "tf2_ros/transform_broadcaster.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"

// class OdomPublisher : public rclcpp::Node
// {
// public:
//     OdomPublisher() : Node("odom_publisher"), px(0), py(0), pz(0), roll(0), pitch(0), yaw(0), t(-1)
//     {
//         state_subscriber_ = this->create_subscription<unitree_go::msg::SportModeState>(
//             "sportmodestate", 10, std::bind(&OdomPublisher::state_callback, this, std::placeholders::_1));
//         odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_convert", 10);
//         tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
//         timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdomPublisher::publish_odometry, this));
//     }

// private:
//     void state_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
//     {
//         if (t < 0)
//         {
//             px = msg->position[0];
//             py = msg->position[1];
//             pz = msg->position[2];
//             roll = msg->imu_state.rpy[0];
//             pitch = msg->imu_state.rpy[1];
//             yaw = msg->imu_state.rpy[2];
//             RCLCPP_INFO(this->get_logger(), "Initial Pose: %f, %f, %f, %f, %f, %f", px, py, pz, roll, pitch, yaw);
//             t = 0;
//         }
//     }

//     void publish_odometry()
//     {
//         if (t >= 0)
//         {
//             auto now = this->get_clock()->now();
//             auto odom_msg = nav_msgs::msg::Odometry();
//             odom_msg.header.stamp = now;
//             odom_msg.header.frame_id = "odom_nav";
//             odom_msg.child_frame_id = "base_footprint";

//             odom_msg.pose.pose.position.x = px;
//             odom_msg.pose.pose.position.y = py;
//             odom_msg.pose.pose.position.z = pz;

//             tf2::Quaternion q;
//             q.setRPY(roll, pitch, yaw);
//             odom_msg.pose.pose.orientation = tf2::toMsg(q);

//             odom_publisher_->publish(odom_msg);

//             // TF transform
//             geometry_msgs::msg::TransformStamped transformStamped;
//             transformStamped.header.stamp = now;
//             transformStamped.header.frame_id = "odom_nav";
//             transformStamped.child_frame_id = "base_footprint";
//             transformStamped.transform.translation.x = px;
//             transformStamped.transform.translation.y = py;
//             transformStamped.transform.translation.z = pz;
//             transformStamped.transform.rotation = tf2::toMsg(q);

//             tf_broadcaster_->sendTransform(transformStamped);

//             RCLCPP_INFO(this->get_logger(), "Publishing Odometry and TF at 10 Hz");
//         }
//     }

//     rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_subscriber_;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
//     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     double px, py, pz;
//     double roll, pitch, yaw;
//     double t;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<OdomPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }
