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

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

struct Point2D {
    double x;
    double y;
};

class MecanumPurePursuit : public rclcpp::Node {
public:
    MecanumPurePursuit()
        : Node("r2_mecanum_pure_pursuit") {
        // odom subscriber (Float32MultiArray: X,Y,Yaw)
        odom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "odom", 10,
            std::bind(&MecanumPurePursuit::odom_callback, this, std::placeholders::_1));

        // cmd_vel publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // パラメータ
        this->declare_parameter<double>("look_ahead_distance", 0.4);   // [m]
        this->declare_parameter<double>("target_speed", 0.5);          // [m/s]
        this->declare_parameter<double>("max_accel", 1.0);             // [m/s^2]  kecepatan smoothing
        this->declare_parameter<double>("stop_radius", 0.15);          // [m] final approach半径
        this->declare_parameter<double>("goal_tolerance", 0.03);       // [m] 完全停止判定 dianggap "sampai"
        this->declare_parameter<double>("path_sampling_step", 0.03);   // [m] パスの密度 kerapatan sampling (analog slider "Kerapatan Sampling Spline")
        this->declare_parameter<bool>("use_smoothing", true);          // Catmull-Rom を使うか
        this->declare_parameter<bool>("dynamic_lookahead", true);      // 終点付近で lookahead を縮小

        this->get_parameter("look_ahead_distance", L_d_base_);
        this->get_parameter("target_speed", target_speed_);
        this->get_parameter("max_accel", max_accel_);
        this->get_parameter("stop_radius", stop_radius_);
        this->get_parameter("goal_tolerance", goal_tolerance_);
        this->get_parameter("path_sampling_step", path_step_);
        this->get_parameter("use_smoothing", use_smoothing_);
        this->get_parameter("dynamic_lookahead", dynamic_lookahead_);

        // サンプル waypoint (x,y) ※実際はKRAIフィールドの座標に合わせて調整
        waypoints_ = {
            {0.0, 0.0}, {-0.3, 0.3}, {0.3, 0.3}, {0.5, 0.5}, {0.0, 0.0}};

        // waypoint から密なパスを生成（スプライン or 直線補間）
        path_ = use_smoothing_ ? buildCatmullRomPath(waypoints_, path_step_)
                                : buildStraightPath(waypoints_, path_step_);

        follow_idx_ = 0;
        vx_cmd_ = 0.0;
        vy_cmd_ = 0.0;
        finished_ = false;
        last_time_ = this->now();

        RCLCPP_INFO(get_logger(), "Path generated: %zu points (smoothing=%s)",
                    path_.size(), use_smoothing_ ? "true" : "false");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::vector<Point2D> waypoints_;
    std::vector<Point2D> path_;   // waypoint を密にした実走行パス
    size_t follow_idx_;           // 直近で使った path インデックス（後退探索防止）
    bool finished_;

    double X_ = 0.0;
    double Y_ = 0.0;
    double yaw_ = 0.0;

    double L_d_base_ = 0.4;
    double target_speed_ = 0.5;
    double max_accel_ = 1.0;
    double stop_radius_ = 0.15;
    double goal_tolerance_ = 0.03;
    double path_step_ = 0.03;
    bool use_smoothing_ = true;
    bool dynamic_lookahead_ = true;

    double vx_cmd_ = 0.0;
    double vy_cmd_ = 0.0;

    rclcpp::Time last_time_;

    // ---------- Catmull-Rom スプライン ----------
    static Point2D catmullRomPoint(const Point2D &p0, const Point2D &p1,
                                    const Point2D &p2, const Point2D &p3, double t) {
        double t2 = t * t, t3 = t2 * t;
        Point2D out;
        out.x = 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t +
                       (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
                       (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
        out.y = 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t +
                       (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
                       (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
        return out;
    }

    static std::vector<Point2D> buildCatmullRomPath(const std::vector<Point2D> &pts, double step) {
        std::vector<Point2D> result;
        if (pts.size() < 2) return pts;

        std::vector<Point2D> ext;
        ext.push_back(pts.front());
        for (auto &p : pts) ext.push_back(p);
        ext.push_back(pts.back());

        for (size_t i = 0; i + 3 < ext.size(); i++) {
            const Point2D &p0 = ext[i], &p1 = ext[i + 1], &p2 = ext[i + 2], &p3 = ext[i + 3];
            double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);
            int steps = std::max(4, static_cast<int>(std::round(seg_len / step)));
            for (int s = 0; s < steps; s++) {
                double t = static_cast<double>(s) / steps;
                result.push_back(catmullRomPoint(p0, p1, p2, p3, t));
            }
        }
        result.push_back(pts.back());
        return result;
    }

    static std::vector<Point2D> buildStraightPath(const std::vector<Point2D> &pts, double step) {
        std::vector<Point2D> result;
        if (pts.size() < 2) return pts;
        for (size_t i = 0; i + 1 < pts.size(); i++) {
            const Point2D &p0 = pts[i];
            const Point2D &p1 = pts[i + 1];
            double seg_len = std::hypot(p1.x - p0.x, p1.y - p0.y);
            int steps = std::max(2, static_cast<int>(std::round(seg_len / step)));
            for (int s = 0; s < steps; s++) {
                double t = static_cast<double>(s) / steps;
                result.push_back({p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t});
            }
        }
        result.push_back(pts.back());
        return result;
    }

    // ---------- Pure Pursuit: lookahead 点探索 ----------
    // path 上で、ロボット位置から距離が look_ahead 以上になる最初の点を探す
    size_t findLookaheadIndex(const Point2D &robot_pos, double look_ahead, size_t start_idx) {
        for (size_t i = start_idx; i < path_.size(); i++) {
            double d = std::hypot(path_[i].x - robot_pos.x, path_[i].y - robot_pos.y);
            if (d >= look_ahead) return i;
        }
        return path_.size() - 1;
    }

    // 現在位置に最も近い path インデックスを探す（後退探索を防ぐための基準点）
    size_t findClosestIndex(const Point2D &robot_pos) {
        size_t best = follow_idx_;
        double best_d = std::numeric_limits<double>::infinity();
        // 全探索（パスは事前計算済みなので軽量。ロボットが速い場合は探索窓を絞ってもよい）
        for (size_t i = 0; i < path_.size(); i++) {
            double d = std::hypot(path_[i].x - robot_pos.x, path_[i].y - robot_pos.y);
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
        return best;
    }

    void odom_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3) {
            RCLCPP_WARN(get_logger(), "odom data too short");
            return;
        }
        if (path_.empty()) return;

        X_ = msg->data[0];
        Y_ = msg->data[1];
        yaw_ = msg->data[2];

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 0.5) dt = 0.02; // 異常値対策（初回・タイムジャンプ時）

        Point2D robot_pos{X_, Y_};
        const Point2D &final_pt = path_.back();
        double dist_to_final = std::hypot(final_pt.x - X_, final_pt.y - Y_);

        if (finished_) {
            publishCmd(0.0, 0.0, dt);
            return;
        }

        // 後退探索を防ぐため、現在の追従インデックスは単調増加させる
        size_t closest = findClosestIndex(robot_pos);
        if (closest > follow_idx_) follow_idx_ = closest;

        double look_ahead = L_d_base_;
        if (dynamic_lookahead_ && dist_to_final < look_ahead * 2.0) {
            // 終点付近では lookahead を縮小し、行き過ぎ（オーバーシュート）を防ぐ
            look_ahead = std::max(0.08, L_d_base_ * (dist_to_final / (look_ahead * 2.0)));
        }

        double target_x, target_y, target_speed;

        if (dist_to_final <= stop_radius_) {
            // ---- Final Approach: 最終waypointへ直接収束させる ----
            target_x = final_pt.x;
            target_y = final_pt.y;
            target_speed = target_speed_ * std::min(1.0, dist_to_final / stop_radius_);

            if (dist_to_final < goal_tolerance_) {
                finished_ = true;
                publishCmd(0.0, 0.0, dt);
                RCLCPP_INFO(get_logger(), "Goal reached. X:%.3f Y:%.3f", X_, Y_);
                return;
            }
        } else {
            // ---- 通常の Pure Pursuit ----
            size_t la_idx = findLookaheadIndex(robot_pos, look_ahead, follow_idx_);
            follow_idx_ = std::max(follow_idx_, la_idx > 0 ? la_idx - 1 : la_idx); // 基準点は緩やかに前進
            target_x = path_[la_idx].x;
            target_y = path_[la_idx].y;
            target_speed = target_speed_;
        }

        // ロボット座標系に変換（yaw固定 = 並進のみ、回転は行わない）
        double dx = target_x - X_;
        double dy = target_y - Y_;
        double cos_yaw = std::cos(-yaw_);
        double sin_yaw = std::sin(-yaw_);
        double vx = cos_yaw * dx - sin_yaw * dy;
        double vy = sin_yaw * dx + cos_yaw * dy;

        double length = std::hypot(vx, vy);
        double desired_vx = 0.0, desired_vy = 0.0;
        if (length > 1e-3) {
            desired_vx = (vx / length) * target_speed;
            desired_vy = (vy / length) * target_speed;
        }

        // ---- 加速度制限（急発進・急停止を抑制してスムーズにする） ----
        double dvx = desired_vx - vx_cmd_;
        double dvy = desired_vy - vy_cmd_;
        double dv_mag = std::hypot(dvx, dvy);
        double max_dv = max_accel_ * dt;
        if (dv_mag > max_dv && dv_mag > 1e-9) {
            vx_cmd_ += dvx / dv_mag * max_dv;
            vy_cmd_ += dvy / dv_mag * max_dv;
        } else {
            vx_cmd_ = desired_vx;
            vy_cmd_ = desired_vy;
        }

        publishCmd(vx_cmd_, vy_cmd_, dt);

        RCLCPP_INFO(get_logger(),
                    "X:%.3f Y:%.3f | target:(%.2f,%.2f) | L_d:%.2f | vx:%.2f vy:%.2f | dist_final:%.2f",
                    X_, Y_, target_x, target_y, look_ahead, vx_cmd_, vy_cmd_, dist_to_final);
    }

    void publishCmd(double vx, double vy, double /*dt*/) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z = 0.0; // Yaw固定
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MecanumPurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}