#include "custom_interface/action/detail/odom_record__struct.hpp"
#include "custom_interface/action/odom_record.hpp"
#include "custom_interface/srv/detail/find_wall__struct.hpp"
#include "custom_interface/srv/find_wall.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <float.h>
#include <functional>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;
using FindWall = custom_interface::srv::FindWall;
using std::placeholders::_1;

class WallFollower : public rclcpp::Node {
public:
  using OdomRecord = custom_interface::action::OdomRecord;
  using GoalHandleOdom = rclcpp_action::ClientGoalHandle<OdomRecord>;

  explicit WallFollower(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("wall_follower_node", node_options), goal_done_(false) {
    group1 = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = group1;

    timer_cbg = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    this->odom_record_client = rclcpp_action::create_client<OdomRecord>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "record_odom", group1);

    twist_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&WallFollower::laser_callback, this, _1), options1);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&WallFollower::timer_callback, this), timer_cbg);

    this->timer_odom =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&WallFollower::send_goal, this));

    this->laser_right = 0.0;
    this->laser_front = 0.0;
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;
    this->timer_odom->cancel();
    this->goal_done_ = false;

    if (!this->odom_record_client) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->odom_record_client->wait_for_action_server(
            std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = OdomRecord::Goal();

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options =
        rclcpp_action::Client<OdomRecord>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&WallFollower::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&WallFollower::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&WallFollower::result_callback, this, _1);

    auto goal_handle_future =
        this->odom_record_client->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp::CallbackGroup::SharedPtr group1;
  rclcpp::CallbackGroup::SharedPtr timer_cbg;
  rclcpp_action::Client<OdomRecord>::SharedPtr odom_record_client;
  geometry_msgs::msg::Twist twist_msg;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_odom;
  bool goal_done_;
  float laser_right;
  float laser_front;

  void
  goal_response_callback(std::shared_future<GoalHandleOdom::SharedPtr> future) {
    //   const GoalHandleOdom::SharedPtr &goal_handle
    const auto &goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleOdom::SharedPtr,
      const std::shared_ptr<const OdomRecord::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Distance travelled:%f",
                feedback->current_total);
  }

  void result_callback(const GoalHandleOdom::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
    // Access the list of odometry points
    auto odometry_points = result.result->list_of_odoms;

    // Log each point in the list
    for (const auto &point : odometry_points) {
      RCLCPP_INFO(this->get_logger(), "Odometry Point: x=%.2f, y=%.2f;",
                  point.x, point.y);
    }
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->laser_right = msg->ranges[180];
    this->laser_front = msg->ranges[359];
    // RCLCPP_INFO(this->get_logger(), "[FRONT] = '%f'", this->laser_front);
    // RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%f'", this->laser_right);
  }

  void timer_callback() {
    if (this->laser_front < 0.4) {
      this->twist_msg.linear.x = 0.05;
      this->twist_msg.angular.z = 0.8;
    }

    else {
      if (this->laser_right > 0.3) {
        this->twist_msg.linear.x = 0.05;
        this->twist_msg.angular.z = -0.09;
        RCLCPP_INFO(this->get_logger(), "Moving towards wall");
      }

      else if (this->laser_right < 0.2) {
        this->twist_msg.linear.x = 0.05;
        this->twist_msg.angular.z = 0.1;
        RCLCPP_INFO(this->get_logger(), "Moving away from wall");
      }

      else {
        this->twist_msg.linear.x = 0.05;
        this->twist_msg.angular.z = 0.00;
        RCLCPP_INFO(this->get_logger(), "Going straight");
      }
    }
    twist_publisher->publish(this->twist_msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("find_wall_client");
  rclcpp::Client<FindWall>::SharedPtr client =
      node->create_client<FindWall>("find_wall");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto request = std::make_shared<FindWall::Request>();

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /find_wall");
  }

  // rclcpp::spin(std::make_shared<WallFollower>());
  auto wall_follow = std::make_shared<WallFollower>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(wall_follow);
  while (!wall_follow->is_goal_done()) {
    executor.spin_some();
  }
  rclcpp::shutdown();
  return 0;
}