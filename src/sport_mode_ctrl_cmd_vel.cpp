#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

class SportRequest : public rclcpp::Node
{
public:
    SportRequest() : Node("req_sender")
    {
        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&SportRequest::state_callback, this, _1));
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        cmd_vel_suber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&SportRequest::cmd_vel_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&SportRequest::timer_callback, this));

        t = -1; // Running time count
    };

private:
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_suber;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    double t; // Running time count
    double dt = 0.002; // Control time step

    double px0 = 0;  // Initial x position
    double py0 = 0;  // Initial y position
    double yaw0 = 0; // Initial yaw angle

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {

        float vx_cmd_vel = msg->linear.x;
        float vy_cmd_vel = msg->linear.y;
        float vyaw_cmd_vel = msg->angular.z;
        printf("sub_data || lin.x : %f, lin.y : %f, ang.z : %f\n",vx_cmd_vel,vy_cmd_vel,vyaw_cmd_vel);
		sport_req.Move(req,vx_cmd_vel,vy_cmd_vel,vyaw_cmd_vel);

        if(vx_cmd_vel != 0.0 || vy_cmd_vel !=0.0 || vyaw_cmd_vel != 0.0){
        req_puber->publish(req);
        }
    }

    void state_callback(const unitree_go::msg::SportModeState::SharedPtr data)
    {
        if (t < 0)
        {
            px0 = data->position[0];
            py0 = data->position[1];
            yaw0 = data->imu_state.rpy[2];
            
        }
    }

    void timer_callback()
    {
        
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SportRequest>());
    rclcpp::shutdown();
    return 0;
}

