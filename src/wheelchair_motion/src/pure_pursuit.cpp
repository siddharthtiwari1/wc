#include <algorithm>
#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "wheelchair_motion/pure_pursuit.hpp"

namespace wheelchair_motion
{
void PurePursuit::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in PurePursuit::configure");
  }

  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Wheelchair-safe parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".look_ahead_distance",
    rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_look_ahead_distance",
    rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_look_ahead_distance",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity",
    rclcpp::ParameterValue(0.3));  // Safe for wheelchair with passenger
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_velocity",
    rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity",
    rclcpp::ParameterValue(0.8));  // Comfortable turning for passengers
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist",
    rclcpp::ParameterValue(0.8));  // Start slowing down at 0.8m from goal
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance",
    rclcpp::ParameterValue(0.2));

  node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
  node->get_parameter(plugin_name_ + ".min_look_ahead_distance", min_look_ahead_distance_);
  node->get_parameter(plugin_name_ + ".max_look_ahead_distance", max_look_ahead_distance_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".min_linear_velocity", min_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead", use_velocity_scaled_lookahead_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);

  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pure_pursuit/carrot", 1);

  RCLCPP_INFO(logger_, "PurePursuit configured with: lookahead=%.2f, max_vel=%.2f, max_ang=%.2f",
    look_ahead_distance_, max_linear_velocity_, max_angular_velocity_);
}

void PurePursuit::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PurePursuit");
  carrot_pub_.reset();
}

void PurePursuit::activate()
{
  RCLCPP_INFO(logger_, "Activating PurePursuit");
  carrot_pub_->on_activate();
}

void PurePursuit::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PurePursuit");
  carrot_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker *)
{
  auto node = node_.lock();
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if (global_plan_.poses.empty()) {
    RCLCPP_ERROR(logger_, "PurePursuit: Empty Plan!");
    return cmd_vel;
  }

  if (!transformPlan(robot_pose.header.frame_id)) {
    RCLCPP_ERROR(logger_, "PurePursuit: Unable to transform Plan to robot's frame");
    return cmd_vel;
  }

  // Get carrot pose using velocity-scaled lookahead if enabled
  double current_speed = std::hypot(velocity.linear.x, velocity.linear.y);
  double lookahead = look_ahead_distance_;
  if (use_velocity_scaled_lookahead_) {
    lookahead = std::clamp(
      current_speed * 1.5,  // Scale lookahead with speed
      min_look_ahead_distance_,
      max_look_ahead_distance_);
  }

  auto carrot_pose = getCarrotPose(robot_pose, lookahead);
  carrot_pub_->publish(carrot_pose);

  // Calculate distance to goal for approach velocity scaling
  double dist_to_goal = getDistanceToGoal(robot_pose);

  // Calculate the curvature to the look-ahead point
  tf2::Transform carrot_pose_robot_tf, robot_tf, carrot_pose_tf;
  tf2::fromMsg(robot_pose.pose, robot_tf);
  tf2::fromMsg(carrot_pose.pose, carrot_pose_tf);
  carrot_pose_robot_tf = robot_tf.inverse() * carrot_pose_tf;
  tf2::toMsg(carrot_pose_robot_tf, carrot_pose.pose);
  double curvature = getCurvature(carrot_pose.pose);

  // Calculate linear velocity with approach scaling
  double linear_vel = max_linear_velocity_;
  if (dist_to_goal < approach_velocity_scaling_dist_) {
    // Scale velocity linearly as we approach goal
    double scale = dist_to_goal / approach_velocity_scaling_dist_;
    linear_vel = std::max(min_linear_velocity_, scale * max_linear_velocity_);
  }

  // Reduce velocity on tight curves (wheelchair comfort)
  double curvature_scale = 1.0 / (1.0 + std::abs(curvature) * 2.0);
  linear_vel *= curvature_scale;

  // Create and publish the velocity command
  cmd_vel.twist.linear.x = std::clamp(linear_vel, min_linear_velocity_, max_linear_velocity_);
  cmd_vel.twist.angular.z = std::clamp(curvature * linear_vel, -max_angular_velocity_, max_angular_velocity_);

  return cmd_vel;
}

void PurePursuit::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO_STREAM(logger_, "Path received with " << path.poses.size() << " poses");
  RCLCPP_INFO_STREAM(logger_, "Path frame " << path.header.frame_id);
  global_plan_ = path;
}

void PurePursuit::setSpeedLimit(const double &, const bool &){}

geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lookahead_dist)
{
  geometry_msgs::msg::PoseStamped carrot_pose = global_plan_.poses.back();
  for (auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it) {
    double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
    double dy = pose_it->pose.position.y - robot_pose.pose.position.y;
    double distance = std::hypot(dx, dy);
    if (distance > lookahead_dist) {
      carrot_pose = *pose_it;
    } else {
      break;
    }
  }
  return carrot_pose;
}

double PurePursuit::getDistanceToGoal(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.empty()) {
    return 0.0;
  }
  const auto & goal = global_plan_.poses.back();
  double dx = goal.pose.position.x - robot_pose.pose.position.x;
  double dy = goal.pose.position.y - robot_pose.pose.position.y;
  return std::hypot(dx, dy);
}

double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose)
{
  const double carrot_dist =
  (carrot_pose.position.x * carrot_pose.position.x) +
  (carrot_pose.position.y * carrot_pose.position.y);
    
  // Find curvature of circle (k = 1 / R)
  if (carrot_dist > 0.001) {
    return 2.0 * carrot_pose.position.y / carrot_dist;
  } else {
    return 0.0;
  }
}

bool PurePursuit::transformPlan(const std::string & frame)
{
  if(global_plan_.header.frame_id == frame){
    return true;
  }
  geometry_msgs::msg::TransformStamped transform;
  try{
    transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR_STREAM(logger_, "Couldn't transform plan from frame " <<
      global_plan_.header.frame_id << " to frame " << frame);
    return false;
  }
  for(auto & pose : global_plan_.poses){
    tf2::doTransform(pose, pose, transform);
  }
  global_plan_.header.frame_id = frame;
  return true;
}

}  // namespace wheelchair_motion

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wheelchair_motion::PurePursuit, nav2_core::Controller)