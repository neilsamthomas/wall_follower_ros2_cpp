//#include "custom_interface/srv/detail/find_wall__struct.hpp"
#include "custom_interface/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

using FindWall = custom_interface::srv::FindWall;
using std::placeholders::_1;
using std::placeholders::_2;

class FinderServerNode : public rclcpp::Node {
public:
  FinderServerNode() : Node("wall_finder_node") {
    group1 = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = group1;

    wall_service_srv = create_service<FindWall>(
        "find_wall", std::bind(&FinderServerNode::wall_finder_callback, this,
                               std::placeholders::_1, std::placeholders::_2));
    twist_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&FinderServerNode::laser_callback, this,
                  std::placeholders::_1),
        options1);
  }

private:
  rclcpp::CallbackGroup::SharedPtr group1;
  rclcpp::Service<FindWall>::SharedPtr wall_service_srv;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  geometry_msgs::msg::Twist twist_msg;

  float min_range_val;
  size_t min_index;
  bool turn_complete = false;
  bool parallel_complete = false;
  bool approach_complete = false;
  float laser_front;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // get range
    const std::vector<float> &ranges = msg->ranges;
    laser_front = msg->ranges[359];

    // find min value
    auto min_range_iter = std::min_element(ranges.begin(), ranges.end());
    this->min_range_val = *min_range_iter;
    this->min_index = std::distance(ranges.begin(), min_range_iter);
  }

  void turn_to_wall() {
    while (!turn_complete) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turning towards the wall %zu",
                  this->min_index);

      if (this->min_index <= 720 && this->min_index >= 362) {
        this->twist_msg.linear.x = 0.0;
        this->twist_msg.angular.z = 0.3;
      }

      else if (this->min_index > 0 && this->min_index <= 356) {
        this->twist_msg.linear.x = 0.0;
        this->twist_msg.angular.z = -0.3;
      } else if (this->min_index <= 362 && this->min_index > 357) {
        this->twist_msg.linear.x = 0.0;
        this->twist_msg.angular.z = 0.0;
        turn_complete = true;
      }
      twist_publisher->publish(twist_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void approach_wall() {
    while (!approach_complete) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approaching wall %f",
                  this->laser_front);
      if (laser_front < 0.28) {
        this->twist_msg.linear.x = -0.03;
        this->twist_msg.angular.z = 0.0;
      } else if (laser_front > 0.32) {
        this->twist_msg.linear.x = 0.03;
        this->twist_msg.angular.z = 0.0;
      } else if (0.28 <= laser_front && laser_front <= 0.32) {
        this->twist_msg.linear.x = 0.0;
        this->twist_msg.angular.z = 0.0;
        approach_complete = true;
      }
      twist_publisher->publish(twist_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void parallel_to_wall() {
    
    while (!parallel_complete) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Turning parallel to the wall %zu", this->min_index);
      if ((min_index <= 200) || (220<= min_index && min_index <= 540)) {
        this->twist_msg.linear.x = 0.0;
        this->twist_msg.angular.z = 0.3;
      } 
      else if (540 <= min_index && min_index <= 720) {
        this->twist_msg.linear.x = 0.0;
        this->twist_msg.angular.z = -0.3;
      } 
      else if (200<= min_index && min_index <= 220) {
        this->twist_msg.linear.x = 0.0;
        this->twist_msg.angular.z = 0.0;
        parallel_complete = true;
      }
      twist_publisher->publish(twist_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void stop_movement() {
    this->twist_msg.linear.x = 0.0;
    this->twist_msg.angular.z = 0.0;
    twist_publisher->publish(twist_msg);
  }

  void
  wall_finder_callback(const std::shared_ptr<FindWall::Request> request,
                       const std::shared_ptr<FindWall::Response> response) {
    turn_to_wall();
    approach_wall();
    parallel_to_wall();
    stop_movement();
    response->wallfound = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approach and Align status: %s",
                response->wallfound ? "true" : "false");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  //   rclcpp::spin(std::make_shared<FinderServerNode>());
  std::shared_ptr<FinderServerNode> wall_finder_node =
      std::make_shared<FinderServerNode>();
  //   rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(wall_finder_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
