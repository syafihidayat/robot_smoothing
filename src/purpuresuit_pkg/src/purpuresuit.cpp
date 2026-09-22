/*
R2 メカナム Pure Pursuit ノード（Yaw固定・改良版）
- Catmull-Rom スプラインによるパス平滑化（オプション）
- 本来の Pure Pursuit：lookahead 距離に基づく先読み点探索
- 加速度制限（急な速度変化を抑制）
- Final Approach モード（終点付近では直接追従に切替え、精度重視）

Copyright (c) 2025 RRST-NHK-Project
*/

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

struct Point2D
{
    double x;
    double y;
};

class PurepursuitPathNode : public rclcpp::Node
{
public:
    PurepursuitPathNode() : Node("pure_pursuit_path_node")
    {

        path_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/path_points", 10, 
            std::bind(&PurepursuitPathNode::pose_callback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            std::bind(&PurepursuitPathNode::odom_callback, this, std::placeholders::_1));

        target_pub = this->create_publisher<geometry_msgs::msg::Point>("/pose", 10);

        path_finished_pub = this->create_publisher<std_msgs::msg::Bool>("path_finished", 10);

        speed_pub = this->create_publisher<std_msgs::msg::Float32>("waypoint_speed", 10);

        this->declare_parameter<double>("look_ahead_distance", 0.4); // meter
        this->declare_parameter<double>("stop_radius", 0.15); // meter
        this->declare_parameter<double>("goal_tolerance", 0.03); // meter
        this->declare_parameter<double>("path_sampling_step", 0.03); // meter
        this->declare_parameter<bool>("use_smoothing", true); 
        this->declare_parameter<bool>("dynamic_lookahead", true);
        this->declare_parameter<double>("publish_rate_hz", 50.0); // hz

        this->declare_parameter<bool>("adaptive_lookahead", false);
        this->declare_parameter<double>("lookahead_speed_gain", 0.3);
        this->declare_parameter<double>("look_ahead_max", 1.0);

        this->declare_parameter<bool>("curvature_speed_regulation", true);
        this->declare_parameter<double>("max_speed", 1.0);
        this->declare_parameter<double>("min_speed_corner", 0.3);
        this->declare_parameter<double>("curvature_lookahead_pts", 8.0);
        this->declare_parameter<double>("curvature_gain", 1.0);



        //_____________________________________________________NOTE_________________________________________________//
            //look_ahead_distance: Jarak "pandang ke depan" Pure Pursuit — seberapa jauh titik target yang dikejar dari posisi robot saat ini di sepanjang path. 
            // - Kecil (misal 0.15)	Robot lebih "nempel" ke kurva, akurat, tapi bisa gemetar/kurang smooth di tikungan tajam
            // - Besar (misal 0.8)	Gerakan lebih smooth & cepat, tapi robot "motong" tikungan (cross-track error lebih besar)

            //stop_radius: Jarak dari titik akhir path di mana robot beralih mode — dari "Pure Pursuit normal" (kejar lookahead point) ke "Final Approach" (kejar titik akhir langsung).
            // - Kecil (misal 0.05)	Mode "presisi" aktif belakangan (baru mepet ke tujuan), robot lebih lama di mode Pure Pursuit biasa
            // - Besar (misal 0.3)	Mode presisi aktif dari jauh, robot melambat/lebih hati-hati lebih awal sebelum sampai

            //goal_tolerance: Jarak minimum untuk menganggap robot "benar-benar sampai" dan menghentikan seluruh proses Pure Pursuit.
            // - Kecil (misal 0.01)	Robot harus sangat presisi baru dianggap "sampai" — makin susah/lama tercapai kalau ada noise odometry
            // - Besar (misal 0.1)	Robot dianggap "sampai" walau masih agak jauh dari titik sebenarnya — kurang presisi tapi lebih cepat "selesai"

            //path_sampling_step: Kerapatan titik-titik hasil sampling spline Makin kecil nilainya, makin rapat titik yang dihasilkan sepanjang kurva 
            // (path lebih halus & Pure Pursuit lebih akurat cari lookahead point), tapi makin banyak titik yang harus disimpan & dicek tiap loop (sedikit lebih berat komputasi).
            
            //use_smoothing: Toggle Catmull-Rom spline on/off. 
            // - true = path di-generate lewat kurva Catmull-Rom (mulus). 
            // - false = path cuma garis lurus antar waypoint

            //dynamic_lookahead: Toggle apakah look_ahead_distance mengecil otomatis saat mendekati titik akhir path.
            // - true: begitu robot dekat titik akhir (jaraknya < 2x lookahead distance), lookahead mengecil bertahap — mencegah robot "overshoot"/lewat dari titik akhir karena masih ngejar titik yang jauh di depan.
            // - false: lookahead tetap konstan sepanjang path, termasuk pas sudah dekat titik akhir — bisa bikin robot susah berhenti presisi (karena tetap ngejar titik yang "melewati" tujuan sebenarnya).

            // publish_rate_hz: Seberapa sering node ini menghitung ulang & publish titik target baru ke /pose (lewat timer_ di constructor).
            // - Rendah (misal 10Hz)	Update target lebih jarang, gerakan robot bisa terasa "patah-patah" karena target-nya update lambat dibanding kecepatan robot
            // - Tinggi (misal 100Hz)	Update lebih sering, gerakan lebih smooth, tapi lebih banyak beban CPU & traffic topic


            // curvature_speed_regulation: on/off fitur ini
            // max_speed: kecepatan target di jalur lurus (curvature ~0)
            // min_speed_corner: batas bawah kecepatan, robot tidak akan lebih pelan dari ini walau kurva sangat tajam
            // curvature_lookahead_pts: jendela pengecekan curvature (dalam jumlah titik path, bukan meter)
            // curvature_gain: makin besar, makin cepat/agresif penurunan kecepatan saat curvature naik




        this->get_parameter("look_ahead_distance", L_d_base);
        this->get_parameter("stop_radius", stop_radius);
        this->get_parameter("goal_tolerance", goal_tolerance);
        this->get_parameter("path_sampling_step", path_step);
        this->get_parameter("use_smoothing", use_smoothing);
        this->get_parameter("dynamic_lookahead", dynamic_lookahead);

        this->get_parameter("adaptive_lookahead", adaptive_lookahead);
        this->get_parameter("lookahead_speed_gain", lookahead_speed_gain);
        this->get_parameter("look_ahead_max", look_ahead_max);

        this->get_parameter("curvature_speed_regulation", curvature_speed_regulation);
        this->get_parameter("max_speed", max_speed);
        this->get_parameter("min_speed_corner", min_speed_corner);
        this->get_parameter("curvature_lookahead_pts", curvature_lookahead_pts);
        this->get_parameter("curvature_gain", curvature_gain);

        double rate_hz;
        this->get_parameter("publish_rate_hz", rate_hz);
        auto period = std::chrono::duration<double>(1.0 / rate_hz);

        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&PurepursuitPathNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "Pure Pursuit path node ready. Waiting for /path_points ...");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr path_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_finished_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    


    std::vector<Point2D> path;
    size_t follow_idx = 0;
    bool has_path = false;
    bool finished = false;
    bool odom_received = false;

    double X_ = 0.0;
    double Y_ = 0.0;
    double current_speed = 0.0;

    double L_d_base = 0.4;
    double stop_radius = 0.15;
    double goal_tolerance = 0.03;
    double path_step = 0.03;
    bool use_smoothing = true;
    bool dynamic_lookahead = true;

    bool adaptive_lookahead = false;
    double lookahead_speed_gain = 0.3;
    double look_ahead_max = 1.0;

    bool curvature_speed_regulation = true;
    double max_speed = 1.0;
    double min_speed_corner = 0.3;
    double curvature_lookahead_pts = 8.0;
    double curvature_gain = 1.0;

    //....................................................Catmull-Rom Spline Smoothing Function....................................................

    static Point2D catmullRomPoint(const Point2D &p0, const Point2D &p1, const Point2D &p2, const Point2D &p3, double t)
    {
        double t2 = t * t;
        double t3 = t2 * t;

        Point2D out;

        out.x = 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t +
                       (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
                       (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
        out.y = 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t +
                       (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
                       (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
        return out;
    }

    static std::vector<Point2D> buildCatmullRomPath(const std::vector<Point2D> &pts, double step)
    {
        std::vector<Point2D> result;
        if (pts.size() < 2)
            return pts;

        std::vector<Point2D> ext;
        ext.push_back(pts.front());
        for (auto &p : pts)
            ext.push_back(p);
        ext.push_back(pts.back());

        for (size_t i = 0; i + 3 < ext.size(); i++)
        {
            const Point2D &p0 = ext[i], &p1 = ext[i + 1], &p2 = ext[i + 2], &p3 = ext[i + 3];
            double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);
            int steps = std::max(4, static_cast<int>(std::round(seg_len / step)));
            for (int s = 0; s < steps; s++)
            {
                double t = static_cast<double>(s) / steps;
                result.push_back(catmullRomPoint(p0, p1, p2, p3, t));
            }
        }

        result.push_back(pts.back());
        return result;
    }

    static std::vector<Point2D> buildStraightPath(const std::vector<Point2D> &pts, double step)
    {
        std::vector<Point2D> result;
        if(pts.size() < 2) return pts;
        for(size_t i = 0; i+1 < pts.size(); i++)
        {
            const Point2D &p0 = pts[i];
            const Point2D &p1 = pts[i + 1];
            double seg_len = std::hypot(p1.x - p0.x, p1.y - p0.y);
            int steps = std::max(2, static_cast<int>(std::round(seg_len / step)));
            for(int s = 0; s < steps; s++){
                double t = static_cast<double>(s) / steps;
                result.push_back({p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t});
            }
        }

        result.push_back(pts.back());
        return result;
    }

    //...........................................................Pure Pursuit.........................................................................

    size_t findLookaheadIndex(const Point2D &robot_pos, double look_ahead, size_t start_idx)
    {
        for (size_t i = start_idx; i < path.size(); i++)
        {
            double d = std::hypot(path[i].x - robot_pos.x, path[i].y - robot_pos.y);
            if (d >= look_ahead)
                return i;
        }

        return path.empty() ? 0 : path.size() - 1;
    }

    size_t findClosestIndex(const Point2D &robot_pos)
    {
        size_t best = follow_idx;
        double best_d = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < path.size(); i++)
        {
            double d = std::hypot(path[i].x - robot_pos.x, path[i].y - robot_pos.y);
            if (d < best_d)
            {
                best_d = d;
                best = i;
            }
        }
        return best;
    }

    double estimateCurvatureAt(size_t idx)
    {
        int window = std::max(1, static_cast<int>(curvature_lookahead_pts));
        long i_prev = static_cast<long>(idx) - window;
        long i_next = static_cast<long>(idx) + window;

        if (i_prev < 0 || i_next >= static_cast<long>(path.size()))
            return 0.0;

        const Point2D &a = path[static_cast<size_t>(i_prev)];
        const Point2D &b = path[idx];
        const Point2D &c = path[static_cast<size_t>(i_next)];

        double ab = std::hypot(b.x - a.x, b.y - a.y);
        double bc = std::hypot(c.x - b.x, c.y - b.y);
        double ac = std::hypot(c.x - a.x, c.y - a.y);

        if (ab < 1e-6 || bc < 1e-6 || ac < 1e-6)
            return 0.0;

        double area2 = std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));

        double curvature = (4.0 * (area2 / 2.0)) / (ab * bc* ac);
        return curvature;
    }

    double lookAheadMaxCurvature(size_t start_idx, int check_span)
    {
        double max_c = 0.0;
        size_t end_idx = std::min(path.size(), start_idx + static_cast<size_t>(check_span));
        for(size_t i = start_idx; i < end_idx; i++)
        {
            double c = estimateCurvatureAt(i);
            if(c > max_c)max_c = c;
        }
        return max_c;
    }

    void pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2 || msg->data.size() % 2 != 0)
        {
            RCLCPP_WARN(get_logger(), "path_points data invalid size: %zu", msg->data.size());
            return;
        }

        std::vector<Point2D> waypoints;

        if (odom_received)
            waypoints.push_back({X_, Y_});
        for (size_t i = 0; i + 1 < msg->data.size(); i += 2)
        {
            waypoints.push_back({static_cast<double>(msg->data[i]), static_cast<double>(msg->data[i + 1])});
        }

        path = use_smoothing ? buildCatmullRomPath(waypoints, path_step)
                             : buildStraightPath(waypoints, path_step);

        follow_idx = 0;
        has_path = true;
        finished = false;

        RCLCPP_INFO(get_logger(), "New path received: %zu waypoints -> %zu path points (smoothing=%s)",
                    (msg->data.size() / 2), path.size(), use_smoothing ? "true" : "false");
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        X_ = msg->pose.pose.position.x;
        Y_ = msg->pose.pose.position.y;
        
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        current_speed = std::hypot(vx,vy);
        
        
        odom_received = true;
    }

    void control_loop()
    {
        if (!has_path || !odom_received || finished || path.empty())
            return;

        Point2D robot_pos{X_, Y_};
        const Point2D &final_pt = path.back();

        double dist_to_final = std::hypot(final_pt.x - X_, final_pt.y - Y_);

        size_t closest = findClosestIndex(robot_pos);
        if (closest > follow_idx)
            follow_idx = closest;

        double look_ahead = L_d_base;

        if(adaptive_lookahead)
        {
            look_ahead = L_d_base + lookahead_speed_gain * current_speed;
            look_ahead = std::min(look_ahead, look_ahead_max);
        }


        if (dynamic_lookahead && dist_to_final < look_ahead * 2.0)
        {
            // look_ahead = std::max(0.05, L_d_base * (dist_to_final / (look_ahead * 2.0)));
            look_ahead = std::max(0.05, look_ahead * (dist_to_final / (look_ahead * 2.0)));
        }

        double target_x, target_y;

        if (dist_to_final <= stop_radius)
        {
            target_x = final_pt.x;
            target_y = final_pt.y;

            if (dist_to_final < goal_tolerance)
            {
                finished = true;
                std_msgs::msg::Bool done;
                done.data = true;
                path_finished_pub->publish(done);
                RCLCPP_INFO(get_logger(), "Path finished. X:%.3f Y:%.3f", X_, Y_);
            }
        }
        else
        {
            size_t la_idx = findLookaheadIndex(robot_pos, look_ahead, follow_idx);
            follow_idx = std::max(follow_idx, la_idx > 0 ? la_idx - 1 : la_idx);
            target_x = path[la_idx].x;
            target_y = path[la_idx].y;
        }

        // ---- Curvature-based speed regulation ----
        double regulated_speed = max_speed;
        if(curvature_speed_regulation && !path.empty())
        {
            int span = std::max(4, static_cast<int>(curvature_lookahead_pts) * 3);
            double max_curv = lookAheadMaxCurvature(follow_idx, span);

            regulated_speed = max_speed / (1.0 + curvature_gain * max_curv);
            regulated_speed = std::max(min_speed_corner, std::min(regulated_speed, max_speed));

            std_msgs::msg::Float32 speed_msg;
            speed_msg.data = static_cast<float>(regulated_speed);
            speed_pub->publish(speed_msg);
        }

        geometry_msgs::msg::Point target_msg;
        target_msg.x = target_x;
        target_msg.y = target_y;
        target_msg.z = 0.0;
        target_pub->publish(target_msg);

        // RCLCPP_INFO(get_logger(),
        //             "pos(%.2f,%.2f) -> target(%.2f,%.2f) | L_d:%.2f | dist_final:%.2f",
        //             X_, Y_, target_x, target_y, look_ahead, dist_to_final);

        RCLCPP_INFO(get_logger(),
            "pos(%.2f,%.2f) -> target(%.2f,%.2f) | L_d:%.2f | dist_final:%.2f | speed_cmd:%.2f",
            X_, Y_, target_x, target_y, look_ahead, dist_to_final, regulated_speed);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurepursuitPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}