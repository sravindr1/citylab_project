#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {

public:
  Patrol() : Node("robot_patrol_node") {

    // Create a reentrant callback group
    auto reentrant_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Subscription options to use this callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = reentrant_group;

    laser_scan_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Patrol::laser_callback, this, std::placeholders::_1),
            sub_options);

    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer options to use the same callback group
    rclcpp::CallbackGroup::SharedPtr timer_group = reentrant_group;

    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_group);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex scan_mutex_; // Protects latest_scan_
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  double direction_{0.0};
  double max_distance_{0.0};
  bool turning_locked_{false};

  void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store latest scan thread-safely
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
  }

  void timer_callback() {
    // Take a thread-safe copy of latest_scan_
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan = latest_scan_;
    }
    if (!scan) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No LaserScan data yet");
      return;
    }

    // if Laser scan data available, calculate front_distance
    double angle_min = scan->angle_min;
    double angle_increment = scan->angle_increment;

    // Index for front
    int front_index = static_cast<int>((0 - angle_min) / angle_increment);
    float front_distance = scan->ranges[front_index];
    geometry_msgs::msg::Twist cmd;

    if (!this->turning_locked_) {

      if (front_distance > 0.35) {
        // Clear path: go straight
        cmd.linear.x = 0.1;
        cmd.angular.z = 0.0;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Moving forward");
      } else {
        // Lock into turning mode
        this->turning_locked_ = true;

        // Search within ±90°
        double left_angle = M_PI / 2;
        double right_angle = -M_PI / 2;

        int start_index = std::max(
            0, static_cast<int>((right_angle - angle_min) / angle_increment));
        int end_index = std::min(
            static_cast<int>((left_angle - angle_min) / angle_increment),
            static_cast<int>(scan->ranges.size() - 1));

        double max_distance = 0.0;
        int max_distance_index = -1;

        for (int i = start_index; i <= end_index; ++i) {
          double r = scan->ranges[i];
          if (std::isfinite(r) && r > max_distance) {
            max_distance = r;
            max_distance_index = i;
          }
        }
        this->max_distance_ = max_distance;

        if (max_distance_index >= 0) {
          this->direction_ = angle_min + angle_increment * max_distance_index;
          RCLCPP_INFO(
              this->get_logger(),
              "Obstacle ahead! Turning to safe direction %.3f rad (%.2f m)",
              this->direction_, this->max_distance_);
          cmd.linear.x = 0.0;
          cmd.angular.z = this->direction_ / 2;
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "Obstacle ahead! No safe direction found. Stopping.");
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.0;
        }
      }
    } else { // this->turning_locked_ is true

      if (front_distance >=
          this->max_distance_ -
              0.1) { // frontdistance >
                     // max_distance -0.01, time to stop turning

        // RCLCPP_INFO(this->get_logger(), "Here");
        // has turned enough , move linear
        this->turning_locked_ = false;
        // Stop and then move in next iteration
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
      } else {

        // RCLCPP_INFO(this->get_logger(), "Turning");
        cmd.linear.x = 0.0;
        cmd.angular.z = this->direction_ / 2;
      }
    }

    cmd_vel_publisher_->publish(cmd);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Patrol>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
