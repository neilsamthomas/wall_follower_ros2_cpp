#include "custom_interface/action/odom_record.hpp"
#include "geometry_msgs/msg/detail/point32__struct.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class OdomRecordServer : public rclcpp::Node {
public:
  using OdomRecord = custom_interface::action::OdomRecord;
  using GoalHandleOdom = rclcpp_action::ServerGoalHandle<OdomRecord>;

  explicit OdomRecordServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("odom_recorder_server", options) {

    callback_group1 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group1;

    twist_pub =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    using namespace std::placeholders;
    this->odom_action_server = rclcpp_action::create_server<OdomRecord>(
        this, "record_odom",
        std::bind(&OdomRecordServer::handle_goal, this, _1, _2),
        std::bind(&OdomRecordServer::handle_cancel, this, _1),
        std::bind(&OdomRecordServer::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), callback_group1);
    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&OdomRecordServer::odom_callback, this,
                  std::placeholders::_1),
        options1);
  }

private:
  rclcpp::CallbackGroup::SharedPtr callback_group1;
  rclcpp_action::Server<OdomRecord>::SharedPtr odom_action_server;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_msg;

  geometry_msgs::msg::Point32 first_odom;
  geometry_msgs::msg::Point32 last_odom;
  double position_x = 0.0;
  double position_y = 0.0;
  double current_dist = 0.0;
  double initial_x = 0.0;
  double initial_y = 0.0;
  double total_distance_travelled = 0.0;
  std::vector<geometry_msgs::msg::Point32> odom_record;

  bool lap_status = false;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_msg = msg;
    position_x = msg->pose.pose.position.x;
    position_y = msg->pose.pose.position.y;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const OdomRecord::Goal>) {
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleOdom> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleOdom> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&OdomRecordServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleOdom> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<OdomRecord::Feedback>();
    auto &current_total = feedback->current_total;
    int a = 0;

    auto result = std::make_shared<OdomRecord::Result>();
    auto move = geometry_msgs::msg::Twist();

    // load first odom values
    odom_callback(latest_odom_msg);
    first_odom.x = position_x;
    first_odom.y = position_y;

    // adding first value to vec odom_record
    odom_record.clear();
    odom_record.push_back(first_odom);
    RCLCPP_INFO(this->get_logger(), "First point added: (%f, %f)", first_odom.x,
                first_odom.y);

    rclcpp::Rate loop_rate(1);

    while (!lap_status) {
      loop_rate.sleep();
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      odom_callback(latest_odom_msg);
      last_odom.x = position_x;
      last_odom.y = position_y;
      odom_record.push_back(last_odom);
      //   RCLCPP_INFO(this->get_logger(), "Next point added added: (%f, %f)",
      //   last_odom.x, last_odom.y);

      if (a == 0) {
        initial_x = first_odom.x;
        initial_y = first_odom.y;
      }
      a++;

      double new_distance = std::sqrt(std::pow(last_odom.x - initial_x, 2) +
                                      std::pow(last_odom.y - initial_y, 2));
      total_distance_travelled += new_distance;
      current_total = total_distance_travelled;
      goal_handle->publish_feedback(feedback);

      double dist_from_start =
          std::sqrt(std::pow(last_odom.x - first_odom.x, 2) +
                    std::pow(last_odom.y - first_odom.x, 2));
      RCLCPP_INFO(this->get_logger(), "Distance from starting point: (%f)",
                  dist_from_start);

      if (current_total >= 5.8 && dist_from_start < 0.5) {
        lap_status = true;
        break;
      }

      initial_x = last_odom.x;
      initial_y = last_odom.y;
    }
    if (rclcpp::ok())
    {
      move.linear.x = 0.0;
      move.angular.z = 0.0;
      twist_pub->publish(move);
      result->list_of_odoms = odom_record;
      goal_handle->succeed(result);
    }

  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto odom_record_srv = std::make_shared<OdomRecordServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(odom_record_srv);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}