
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {

public:
  Patrol() : Node("robot_patrol_node") {

    laser_scan_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,

            std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ =
        this->create_wall_timer(0.1s, std::bind(&Patrol::timer_callback, this));
  }
  double direction_{0.0};

private:
  bool turning_locked_ = false;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
  }

  void timer_callback() {

    if (!latest_scan_) {
      RCLCPP_INFO(this->get_logger(), "LaserScan data not available");
      return; // scan not available yet
    }
    auto scan = latest_scan_;
    double angle_min = scan->angle_min;
    double angle_increment = scan->angle_increment;

    int front_index = static_cast<int>((0 - angle_min) / angle_increment);
    float front_distance = scan->ranges[front_index];
    geometry_msgs::msg::Twist cmd;

    if (!this->turning_locked_) {

      if (front_distance > 0.35) {

        cmd.linear.x = 0.1;
        cmd.angular.z = 0;

        RCLCPP_INFO(this->get_logger(), "Moving Forward");
      } else {
        this->turning_locked_ = true;
        // get data indices to filter
        double left_angle = M_PI / 2;
        double right_angle = -M_PI / 2;

        int start_index = std::max(
            0, static_cast<int>((right_angle - angle_min) / angle_increment));
        int end_index = std::min(
            static_cast<int>((left_angle - angle_min) / angle_increment),
            static_cast<int>(scan->ranges.size() - 1));

        double max_distance = 0;
        int max_distance_index = -1;

        for (int i = start_index; i <= end_index; i++) { // filter data
          double range_data = scan->ranges[i];

          if (std::isfinite(range_data)) { // if data is finite
            if (range_data > max_distance) {
              max_distance = range_data;
              max_distance_index = i;
            }
          }
        }

        if (max_distance_index >= 0) { // condition true if valid data was
                                       // processed in range -90deg to +90 deg

          this->direction_ = angle_min + angle_increment * max_distance_index;
          RCLCPP_INFO(
              this->get_logger(),
              "Obstacle detected! Turn to safe distance %0.2f m at angle "
              "%0.3f rad",
              max_distance, this->direction_);
          cmd.linear.x = 0.0;
          cmd.angular.z = this->direction_ / 2;
        }

        else {
          // no safe distance within processed data, so stop;
          RCLCPP_INFO(this->get_logger(),
                      "Obstacle detected! Stop, no safe angle turn available ");
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.0;
        }
      }
    }

    if (this->turning_locked_) {
      if (front_distance > 0.35) {
        // Front is clear -> stop turning
        this->turning_locked_ = false;
        cmd.linear.x = 0.1;
        cmd.angular.z = 0.0;
      } else {
        // Keep turning in the previously computed direction
        cmd.linear.x = 0.0;
        cmd.angular.z = this->direction_ / 2;
      }
    }

    cmd_vel_publisher_->publish(cmd);
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto patrolbot = std::make_shared<Patrol>();

  //   patrolbot type is std::shared_ptr<rclcpp::Node>

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(patrolbot);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}