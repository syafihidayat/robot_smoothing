#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>


class waypointPublish : public rclcpp::Node
{
public:
  waypointPublish() : Node("send_pose")
  {
    path_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/path_points", 10);

    path_finished_sub = this->create_subscription<std_msgs::msg::Bool>("path_finished", 10, 
      std::bind(&waypointPublish::path_finished_callback, this, std::placeholders::_1));

    // reached_sub = this->create_subscription<std_msgs::msg::Bool>(
    //     "/target_reached", 10,
    //     std::bind(&waypointPublish::reached_callback, this, std::placeholders::_1));

    start_sub = this->create_subscription<std_msgs::msg::Bool>("robot_start", 10,
                  std::bind(&waypointPublish::start_callback, this, std::placeholders::_1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                                                                  [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                                  {
                                                                    current_robot_x = msg->pose.pose.position.x;
                                                                    current_robot_y = msg->pose.pose.position.y;
                                                                  });

    waypoint = {

   

        {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0}
        // {0.0, 1.0, 0.0}
        // {1.5, -1.15}

      };

    // current_waypoint_index = 0;
    // reached_latched = false;
    
    all_completed = false;
    start_triggered = false;                  

    RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Ready, menunggu robot_start...");
    // send_waypoint(); // ← JANGAN langsung jalan di sini, harus nunggu tombol start (topic "robot_start")
  }

private:

  // WP_STATE state = WP_STATE::MOVING;

  bool all_completed = false;
  double current_robot_x = 0.0, current_robot_y = 0.0;
  bool start_triggered = false;
  
  
  // bool reached_latched = false;
  // double exit_x = 0.0, exit_y = 0.0;
  // bool exit_pos_received = false;
  // bool exit_pos_pushed = false;
  // bool retry_mode = false;

  // size_t current_waypoint_index;

  // ===================== SEND WAYPOINT =====================
  void send_all_waypoint()
  {

    if(waypoint.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Waypoint list kosong, tidak ada yang dikirim");
      return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(waypoint.size() * 2);

    for(auto &wp : waypoint)
    {
      msg.data.push_back(static_cast<float>(std::get<0>(wp)));
      msg.data.push_back(static_cast<float>(std::get<1>(wp)));
    }

    path_pub->publish(msg);
    all_completed = false;

    RCLCPP_INFO(this->get_logger(), "Path terkirim: %zu waypoint sekaligus", waypoint.size());

    // if (current_waypoint_index >= waypoint.size())
    // {
    //   all_completed = true;
    //   state = WP_STATE::COMPLETE;
    //   printf("ALL WAYPOINT COMPLETE\n");
    //   return;
    // }

    // geometry_msgs::msg::Point point;
    // point.x = std::get<0>(waypoint[current_waypoint_index]);
    // point.y = std::get<1>(waypoint[current_waypoint_index]);
    // point.z = std::get<2>(waypoint[current_waypoint_index]);

    // path_pub->publish(point);
    // reached_latched = false;


    // RCLCPP_INFO(this->get_logger(), "Sending waypoint %ld (%.2f, %.2f)", current_waypoint_index, point.x, point.y);
  }

  // ===================== ADVANCE (ONLY ONE ENTRY POINT) =====================
  // void advance_waypoint()
  // {
  //   current_waypoint_index++;
  //   state = WP_STATE::MOVING;

  //   send_waypoint();
  // }

  // ===================== START CALLBACK (tombol start GUI) =====================
  void start_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;
    if (start_triggered)
      return; // anti double trigger, jaga2 kalau topic dipublish berkali2

    start_triggered = true;
    // retry_mode = false; 

    RCLCPP_INFO(this->get_logger(), "ROBOT_START DITERIMA -> mulai kirim waypoint pertama");
    send_all_waypoint();
  }

  void path_finished_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(all_completed)return;
    if(!msg->data)return;

    all_completed = true;
    RCLCPP_INFO(this->get_logger(), "SELURUH PATH SELESAI DILEWATI");
  }

  // ===================== REACHED CALLBACK =====================
  // void reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
  // {
  //   if (all_completed)
  //     return;
  //   if (!msg->data)
  //     return;
  //   if (reached_latched)
  //     return; // anti double trigger

  //   reached_latched = true;

  //   RCLCPP_INFO(this->get_logger(), "WAYPOINT REACHED: %ld", current_waypoint_index);
  //   if (retry_mode)
  //   {
  //     advance_waypoint();
  //     return;
  //   }


  //   // WP lainnya langsung lanjut
  //   advance_waypoint();
  // }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr path_pub;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_finished_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub;


  std::vector<std::tuple<double, double, double>> waypoint;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypointPublish>());
  rclcpp::shutdown();
  return 0;
}
