#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include "robot_main_pkg/pid.hpp"
#include "robot_main_pkg/convertion.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


PID omni_distance;
PID omni_angular;

class Movement : public rclcpp::Node
{

public:
  Movement() : Node("Movement_Point")
  {
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/omni_cont/cmd_vel", 10);

    reached_pub = this->create_publisher<std_msgs::msg::Bool>("/target_reached", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
    std::bind(&Movement::odom_callback, this, std::placeholders::_1));

    pose_sub = this->create_subscription<geometry_msgs::msg::Point>("/pose", 10,
    std::bind(&Movement::pose_callback, this, std::placeholders::_1));


    timer_ = this->create_wall_timer(std::chrono::milliseconds(60), std::bind(&Movement::control_loop, this));

    start_received = false;
    target_received = false;
    current_state = WAITING_FOR_TARGET;
    prevT = this->now().seconds();

    float kp, ki, kd;
    float kpT, kiT, kdT;

    this->declare_parameter("kp", 0.75);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);

    this->declare_parameter("kpT", 1.2);
    this->declare_parameter("kiT", 0.0);
    this->declare_parameter("kdT", 0.0);

    this->get_parameter("kp", kp);
    this->get_parameter("ki", ki);
    this->get_parameter("kd", kd);

    this->get_parameter("kpT", kpT);
    this->get_parameter("kiT", kiT);
    this->get_parameter("kdT", kdT);

    this->declare_parameter("desired_linear_vel", 0.8);
    this->declare_parameter("max_angular_vel", 0.8);
    
    this->get_parameter("desired_linear_vel", desired_linear_vel);
    this->get_parameter("max_angular_vel", max_angular_vel);

    omni_distance.setBaseParam(kp, ki, kd);
    omni_angular.setHeadingParam(kpT, kiT, kdT);

    RCLCPP_INFO(this->get_logger(), "waiting for targets");
  }

private:
  enum State
  {
    WAITING_FOR_TARGET,
    MOVING_TO_TARGET,
    TARGET_REACHED
  };

  float desired_linear_vel, max_angular_vel;

  int stage1_target_total = 3;
  int stage1_target_count = 0;

  bool stage1_complated = false;
  double heading = 0.0;

  double startX,startY;
  double targetX,targetY;
  double currentX,currentY;

  bool start_received;
  bool target_received;

  State current_state;
  double prevT;
  bool target_reached_flag = false;

  double convertion(const nav_msgs::msg::Odometry &odom_robot)
  {
    Convertion cornvert;

    Convertion::Quaternion robot_quat = {
        odom_robot.pose.pose.orientation.w,
        odom_robot.pose.pose.orientation.x,
        odom_robot.pose.pose.orientation.y,
        odom_robot.pose.pose.orientation.z,
    };

    double odom_robot_yaw,odom_robot_pitch,odom_robot_roll;
    cornvert.quat_to_eular(robot_quat, odom_robot_yaw, odom_robot_pitch, odom_robot_roll);

    odom_robot_yaw = -odom_robot_yaw;

    return odom_robot_yaw;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Convertion cnvrt;

    Convertion::Quaternion robot_q = {
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
    };

    double yaw,pitch,roll;
    cnvrt.quat_to_eular(robot_q, yaw, pitch, roll);

    heading = yaw;

    odom_robot_msg = *msg;

    currentX = msg->pose.pose.position.x;
    currentY = msg->pose.pose.position.y;
    if(!start_received)
    {
      startX = currentX;
      startY = currentY;
      start_received = true;
    }
  }

  void pose_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if(current_state == WAITING_FOR_TARGET || current_state == TARGET_REACHED)
    {
      targetX = msg->x;
      targetY = msg->y;
      target_received = true;
      current_state = MOVING_TO_TARGET;

      RCLCPP_INFO(this->get_logger(), "New target STAGE1 received: (%.2f, %.2f)", targetX, targetY);
      RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)", currentX, currentY);
    }

  }

  void control_loop()
  {
    if(!start_received)
      return;

    if((current_state == WAITING_FOR_TARGET || current_state == MOVING_TO_TARGET) && !target_received)
      return;

      double currT = this->now().seconds();
      float deltaT = currT - prevT;

      geometry_msgs::msg::Twist cmd;

      switch(current_state)
      {
      case WAITING_FOR_TARGET:
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

        if(target_received)
        {
          current_state = MOVING_TO_TARGET;
          RCLCPP_INFO(this->get_logger(), "🎯 Starting movement to target %d/%d",stage1_target_count + 1, stage1_target_total);
        }
        break;

      
      case MOVING_TO_TARGET:
      {
        double dx = targetX - currentX;
        double dy = targetY - currentY;

        double odom_robot_yaw = convertion(odom_robot_msg);
        double theta = 0 - odom_robot_yaw;

        double distance = std::sqrt(dx * dx + dy * dy);
        double angle = std::atan2(dy, dx);

        float control_distance = omni_distance.control_base(distance, desired_linear_vel);
        float control_angle = omni_angular.control_base_rotation(theta, max_angular_vel);


        if(distance > 0.03)
        {
          cmd.linear.x = control_distance * std::cos(angle);
          cmd.linear.y = control_distance * std::sin(angle);
          cmd.angular.z = control_angle ;

          RCLCPP_INFO(this->get_logger(), "move robot");

        }
        else
        {
          cmd.linear.x = 0;
          cmd.linear.y = 0;
          cmd.angular.z = 0;

          current_state = TARGET_REACHED;
          stage1_target_count++;

          std_msgs::msg::Bool reached_msg;
          reached_msg.data = true;
          reached_pub->publish(reached_msg);

          if(stage1_target_count >= stage1_target_total)
          {
            stage1_complated = true;
            RCLCPP_INFO(this->get_logger(), "target reached");
            RCLCPP_INFO(this->get_logger(), "STAGE1 COMPLATED");

          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "wait for next target");

          }

          break;
        }
      }

      case TARGET_REACHED:

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

        break;
      }

        cmd_pub->publish(cmd);
      }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_sub;

  nav_msgs::msg::Odometry odom_robot_msg;

  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Movement>());
  rclcpp::shutdown();
  return 0;
}