#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>


class waypointPublish : public rclcpp::Node
{
public:
  waypointPublish() : Node("send_pose")
  {
    pose_pub = this->create_publisher<geometry_msgs::msg::Point>("/pose", 10);

    reached_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/target_reached", 10,
        std::bind(&waypointPublish::reached_callback, this, std::placeholders::_1));

    start_sub = this->create_subscription<std_msgs::msg::Bool>("robot_start", 10,
                  std::bind(&waypointPublish::start_callback, this, std::placeholders::_1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                                                                  [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                                  {
                                                                    current_robot_x = msg->pose.pose.position.x;
                                                                    current_robot_y = msg->pose.pose.position.y;
                                                                  });

    speed_pub = this->create_publisher<std_msgs::msg::Float32>("/waypoint_speed", 10);

    waypoint = {

        // {0.2, 0.0}
        // {0.0, 1.0}

        {0.0, 1.1, 0.0},
        {1.1, 1.1, 0.0}
        // {0.0, 1.1, 0.0, 0.1}
        // {1.5, -1.15}

      };

    current_waypoint_index = 0;
    reached_latched = false;                  
    all_completed = false;

    RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Ready, menunggu robot_start...");
    // send_waypoint(); // ← JANGAN langsung jalan di sini, harus nunggu tombol start (topic "robot_start")
  }

private:
  enum class WP_STATE
  {
    MOVING,
    WAIT_IR,
    WAIT_LIFTER,
    WAIT_STAGE2_LIFTER,
    COMPLETE
  };

  WP_STATE state = WP_STATE::MOVING;

  bool reached_latched = false;
  bool all_completed = false;
  double exit_x = 0.0, exit_y = 0.0;
  double current_robot_x = 0.0, current_robot_y = 0.0;
  bool exit_pos_received = false;
  bool exit_pos_pushed = false;
  bool retry_mode = false;
  bool start_triggered = false;

  size_t current_waypoint_index;

  // ===================== SEND WAYPOINT =====================
  void send_waypoint()
  {
    if (current_waypoint_index >= waypoint.size())
    {
      all_completed = true;
      state = WP_STATE::COMPLETE;
      printf("ALL WAYPOINT COMPLETE\n");
      return;
    }

    geometry_msgs::msg::Point point;
    point.x = std::get<0>(waypoint[current_waypoint_index]);
    point.y = std::get<1>(waypoint[current_waypoint_index]);
    point.z = std::get<2>(waypoint[current_waypoint_index]);

    pose_pub->publish(point);
    reached_latched = false;

    // std_msgs::msg::Float32 speed_msg;
    // speed_msg.data = static_cast<float>(std::get<2>(waypoint[current_waypoint_index]));
    // speed_pub->publish(speed_msg);


    RCLCPP_INFO(this->get_logger(), "Sending waypoint %ld (%.2f, %.2f)", current_waypoint_index, point.x, point.y);
  }

  // ===================== ADVANCE (ONLY ONE ENTRY POINT) =====================
  void advance_waypoint()
  {
    current_waypoint_index++;
    state = WP_STATE::MOVING;

    send_waypoint();
  }

  // ===================== START CALLBACK (tombol start GUI) =====================
  void start_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;
    if (start_triggered)
      return; // anti double trigger, jaga2 kalau topic dipublish berkali2

    start_triggered = true;
    retry_mode = false; 

    RCLCPP_INFO(this->get_logger(), "ROBOT_START DITERIMA -> mulai kirim waypoint pertama");
    send_waypoint();
  }

  // ===================== REACHED CALLBACK =====================
  void reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (all_completed)
      return;
    if (!msg->data)
      return;
    if (reached_latched)
      return; // anti double trigger

    reached_latched = true;

    RCLCPP_INFO(this->get_logger(), "WAYPOINT REACHED: %ld", current_waypoint_index);
    if (retry_mode)
    {
      advance_waypoint();
      return;
    }


    // WP lainnya langsung lanjut
    advance_waypoint();
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub;


  std::vector<std::tuple<double, double, double>> waypoint;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypointPublish>());
  rclcpp::shutdown();
  return 0;
}
