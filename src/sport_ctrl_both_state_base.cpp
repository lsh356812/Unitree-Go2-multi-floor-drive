#include <unistd.h>
#include <cmath>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "nav_msgs/msg/path.hpp"  // Include for nav_msgs/Path
#include "std_msgs/msg/string.hpp"  // Include for std_msgs/String
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
        path_suber = this->create_subscription<nav_msgs::msg::Path>(
            "local_path_1", 10, std::bind(&SportRequest::path_callback, this, _1));  // Local path subscription
        control_mode_suber = this->create_subscription<std_msgs::msg::String>(
            "control_mode", 10, std::bind(&SportRequest::control_mode_callback, this, _1));  // Control mode subscription
        p_state_pub = this->create_publisher<std_msgs::msg::String>("/path_state", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&SportRequest::timer_callback, this));
        
        t = -1; // Running time count
    };

private:
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_suber;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_suber;  // Path subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_suber;  // Control mode subscriber
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_state_pub;
    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;
    std::string Control_Mode;  // Global variable for control mode
    std_msgs::msg::String path_state;  // Global variable for path state
    // std::vector<PathPoint> path;
    nav_msgs::msg::Path path_topic;
    // path_topic.poses.resize(30);
    
    // PathPoint path_point_tmp;

    double t; // Running time count
    double dt = 0.1; // Control time step

    double px0 = 0;  // Initial x position
    double py0 = 0;  // Initial y position
    double yaw0 = 0; // Initial yaw angle
    double current_x = 0.;
    double current_y = 0.;
    double target_pose_x = 0.;
    double target_pose_y = 0.;
    int Control_flag = 0; // 1 -> move start, 2 -> move ing, 3 -> move_complete
    double tolerance = 0.45;
    int path_flag = 0; // path num 1-> path_1 complete // 2-> path_2 complete // 3-> path_3 complete
    int timer_cnt = 0;
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float vx_cmd_vel = msg->linear.x;
        float vy_cmd_vel = msg->linear.y;
        float vyaw_cmd_vel = msg->angular.z;
        printf("sub_data || lin.x : %f, lin.y : %f, ang.z : %f\n",vx_cmd_vel,vy_cmd_vel,vyaw_cmd_vel);
        sport_req.Move(req,vx_cmd_vel,vy_cmd_vel,vyaw_cmd_vel);

        if(Control_Mode == "cmd_vel_mode"){
            if(vx_cmd_vel != 0.0 || vy_cmd_vel !=0.0 || vyaw_cmd_vel != 0.0){
            req_puber->publish(req);
            }
        }
    }

    void state_callback(const unitree_go::msg::SportModeState::SharedPtr data)
    {
        if (t < 0)
        {
            px0 = data->position[0];
            py0 = data->position[1];
            yaw0 = data->imu_state.rpy[2];
            // printf("state_read\n");
            if(path_flag == 2 || path_flag == 5){
                path_state.data = "path_moving";
                p_state_pub->publish(path_state);
            }
            
        }
        if (t > 0 && Control_Mode == "path_mode")
        {
            current_x = data->position[0]-px0;
            current_y = data->position[1]-py0;
            printf("curr_x : %lf, curr_y : %lf, targ_ x : %lf, targ_Y : %lf \n",current_x, current_y, target_pose_x,target_pose_y);
            // printf("error_x : %lf, error_y : %lf \n",current_x+target_pose_x , current_y+target_pose_y);
            
            if(fabs(current_x + target_pose_x) < tolerance){ // && current_y - target_pose_y < tolerancence && fabs(current_y + target_pose_y) < tolera
                printf("Move_complete\n");
                printf("path_flag : %d\n", path_flag);
                Control_flag = 3;
                    t = -1;
                // if(path_flag == 0){
                //     path_flag =1;
                //     path_state.data = "path_1_complete";
                //     p_state_pub->publish(path_state);

                //     Control_flag = 1;
                // }
                // if(path_flag == 1){
                //     path_flag =2;
                //     path_state.data = "path_2_complete";
                //     p_state_pub->publish(path_state);

                //     Control_flag = 1;
                // }
                // if(path_flag == 2){
                //     path_flag =3;
                //     path_state.data = "path_3_complete";
                //     p_state_pub->publish(path_state);

                //     Control_flag = 1;
                // }
                
                // if(path_flag == 3){
                //     path_flag =4;
                //     path_state.data = "path_num_init";
                //     p_state_pub->publish(path_state);

                //     Control_flag = 1;
                // }

                switch(path_flag){
                    case 0:
                    path_flag = 1;
                    path_state.data = "path_1_complete";
                    p_state_pub->publish(path_state);

                    Control_flag = 1;
                    break;

                    case 1:
                    path_flag = 2;
                    path_state.data = "path_2_complete";
                    p_state_pub->publish(path_state);

                    Control_flag = 1;
                    break;

                    case 2:
                    path_flag = 3;
                    path_state.data = "path_3_complete";
                    p_state_pub->publish(path_state);

                    Control_flag = 1;
                    break;

                    case 3:
                    path_flag = 4;
                    path_state.data = "path_num_init";
                    p_state_pub->publish(path_state);

                    Control_flag = 1;
                    break;
                }
            }
            if(path_flag == 2 || path_flag == 5){
                path_state.data = "path_moving";
                p_state_pub->publish(path_state);
            }

        }
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {   
        double time_seg = 0.3;
        
        if(Control_Mode == "path_mode"){
            if(Control_flag == 3 || path_state.data == "path_1_complete" || path_state.data == "path_2_complete" || path_state.data == "path_3_complete"){
                Control_flag = 1;
                printf("new_Path_setup\n");
            }

            
            if(Control_flag == 1){
                path_topic.poses.resize(30);
                t = -1;
                timer_cnt = 0;
                target_pose_x = msg->poses[29].pose.position.x;
                target_pose_y = msg->poses[29].pose.position.y;
                printf("Path_init_complete\n======================\n x: %lf , y: %lf\n", target_pose_x, target_pose_y);


            for (int i = 0; i < 30; i++)
            {
                path_topic.poses[i] = msg->poses[i];
                // time_temp += time_seg;
                // Tacking a sin path in x direction
                // The path is respect to the initial coordinate system
                // float px_local = msg->poses[i].pose.position.x;
                // float py_local = msg->poses[i].pose.position.y;
                // float yaw_local = 0.;
                // float vx_local = 0.01;
                // float vy_local = 0.01;
                // float vyaw_local = 0.;

                // // Convert trajectory commands to the initial coordinate system
                // path_point_tmp.timeFromStart = i * time_seg;
                // path_point_tmp.x = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
                // path_point_tmp.y = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
                // path_point_tmp.yaw = yaw0;
                // if(i < 29){
                // path_point_tmp.vx = vx_local * cos(yaw0) - vy_local * sin(yaw0);//vx_local * cos(yaw0) - vy_local * sin(yaw0);
                // path_point_tmp.vy = vx_local * sin(yaw0) + vy_local * cos(yaw0);//vx_local * sin(yaw0) + vy_local * cos(yaw0);    
                // }
                // else{
                // path_point_tmp.vx = 0;
                // path_point_tmp.vy = 0;
                // }
                // path_point_tmp.vyaw = 0;//vyaw_local;
                // path.push_back(path_point_tmp);
                // printf("Received path with %zu poses\n", msg->poses[i].pose.position.x);
                // printf("Received path with %zu poses\n", msg->poses[i].pose.position.y);
            }
            // sport_req.TrajectoryFollow(req, path);

            // // Publish request messages
            
            // req_puber->publish(req);
            Control_flag = 2;
            }

        
        else if(Control_flag == 2){
            printf("Moveing... \n");
        }
        else if(Control_flag == 3){
            printf("Path Following done.\n");
            Control_Mode = "None";
        }
        }
            // Get request messages corresponding to high-level motion commands
    }

    void control_mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if(Control_Mode != msg->data){
            Control_Mode = msg->data;
            printf("Control mode updated to: %s\n", Control_Mode.c_str());
            Control_flag = 1;
            path_flag = 0;
            if(Control_Mode == "path_mode"){
            path_flag = 0;
            t = -1;
            path_state.data = "path_start";
            p_state_pub->publish(path_state);
            }
        }

    }

    void timer_callback()
    {
        t += dt;
        
        if (t > 0)
        {
            double time_seg = 0.2;
            double time_temp = t - time_seg;

            std::vector<PathPoint> path;
            printf("for_start\n");
            for (int i = 0; i < 30; i++)
            {
                // printf("0\n");
                PathPoint path_point_tmp;
                time_temp += time_seg;
                // Tacking a sin path in x direction
                // The path is respect to the initial coordinate system
                // float px_local = 0.01*time_temp;
                // float py_local = 0;
                float px_local = path_topic.poses[i].pose.position.x;
                float py_local = path_topic.poses[i].pose.position.y;
                float yaw_local = path_topic.poses[i].pose.position.z;
                float vx_local = 0;
                float vy_local = 0;
                float vyaw_local = 0.;
                // printf("1\n");
                // Convert trajectory commands to the initial coordinate system
                path_point_tmp.timeFromStart = i * time_seg;
                path_point_tmp.x = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
                path_point_tmp.y = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
                path_point_tmp.yaw = yaw0 + yaw_local;
                path_point_tmp.vx = 0;//vx_local * cos(yaw0) - vy_local * sin(yaw0);
                path_point_tmp.vy = 0;//vx_local * sin(yaw0) + vy_local * cos(yaw0);
                path_point_tmp.vyaw = 0;//vyaw_local;
                path.push_back(path_point_tmp);
                // printf("for -> t : %lf, time_temp : %lf\n",t , time_temp);
            }
            
            for (int j = 0; j < 30 - 1; ++j) {
                path_topic.poses[j] = path_topic.poses[j + 1];  // 배열의 원소를 앞으로 한 칸씩 이동
            }
                path_topic.poses[29].pose.position.x = target_pose_x;  // 마지막 원소에 첫 번째 원소의 값을 채움
                path_topic.poses[29].pose.position.y = target_pose_y;  // 마지막 원소에 첫 번째 원소의 값을 채움

            // Get request messages corresponding to high-level motion commands
            sport_req.TrajectoryFollow(req, path);
            // Publish request messages
            req_puber->publish(req);
            timer_cnt = timer_cnt + 1;
        }
        else if(Control_flag != 2){
            t = -1;
            // printf("before path sub\n");
        }
        else{
            printf("move done \n");
            timer_cnt = 0;
            printf("init\n");
            
        }
    }
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SportRequest>());
    rclcpp::shutdown();
    return 0;
}


//
// path_point_tmp.timeFromStart = i * time_seg;
//                 path_point_tmp.x = path_topic.poses[i].pose.position.x * cos(yaw0) - path_topic.poses[i].pose.position.y * sin(yaw0) + px0;
//                 path_point_tmp.y = path_topic.poses[i].pose.position.x * sin(yaw0) + path_topic.poses[i].pose.position.y * cos(yaw0) + py0;
//                 path_point_tmp.yaw = yaw0;
//                 path_point_tmp.vx = 0;//vx_local * cos(yaw0) - vy_local * sin(yaw0);
//                 path_point_tmp.vy = 0;//vx_local * sin(yaw0) + vy_local * cos(yaw0);
//                 path_point_tmp.vyaw = 0;//vyaw_local;
//                 path.push_back(path_point_tmp);

//                  for (int j = 0; j < 30 - 1; ++j) {
//                     path_topic.poses[j] = path_topic.poses[j + 1];  // 배열의 원소를 앞으로 한 칸씩 이동
//                 }
//                     path_topic.poses[29].pose.position.x = target_pose_x;  // 마지막 원소에 첫 번째 원소의 값을 채움
//                     path_topic.poses[29].pose.position.y = target_pose_y;  // 마지막 원소에 첫 번째 원소의 값을 채움