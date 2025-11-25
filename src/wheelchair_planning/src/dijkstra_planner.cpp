#include <cmath>
#include <chrono>
#include <algorithm>

#include "wheelchair_planning/dijkstra_planner.hpp"

namespace wheelchair_planning
{

void DijkstraPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(node_->get_logger(),
    "Dijkstra Planner configured. Costmap: %dx%d, resolution: %.3f, frame: %s",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(),
    costmap_->getResolution(), global_frame_.c_str());

  smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_, "smooth_path");
}

void DijkstraPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "CleaningUp plugin %s of type DijkstraPlanner", name_.c_str());
}

void DijkstraPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type DijkstraPlanner", name_.c_str());
}

void DijkstraPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type DijkstraPlanner", name_.c_str());
}

nav_msgs::msg::Path DijkstraPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  // Log start and goal
  RCLCPP_INFO(node_->get_logger(),
    "Planning from (%.2f, %.2f) to (%.2f, %.2f)",
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // Convert to grid coordinates
  GraphNode start_node = worldToGrid(start.pose);
  GraphNode goal_node = worldToGrid(goal.pose);

  RCLCPP_INFO(node_->get_logger(),
    "Grid coords: start(%d, %d) -> goal(%d, %d)",
    start_node.x, start_node.y, goal_node.x, goal_node.y);

  // Validate start and goal are on map
  if (!poseOnMap(start_node)) {
    RCLCPP_ERROR(node_->get_logger(), "Start position is outside costmap!");
    return path;
  }
  if (!poseOnMap(goal_node)) {
    RCLCPP_ERROR(node_->get_logger(), "Goal position is outside costmap!");
    return path;
  }

  // Check if start or goal are in obstacles (cost >= 253 is lethal)
  unsigned char start_cost = costmap_->getCost(start_node.x, start_node.y);
  unsigned char goal_cost = costmap_->getCost(goal_node.x, goal_node.y);

  RCLCPP_INFO(node_->get_logger(), "Start cost: %d, Goal cost: %d", start_cost, goal_cost);

  if (goal_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    RCLCPP_ERROR(node_->get_logger(), "Goal is in obstacle (cost=%d)!", goal_cost);
    return path;
  }

  // 4-connected movement directions
  std::vector<std::pair<int, int>> explore_directions = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}
  };

  std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
  std::vector<std::vector<bool>> visited(
    costmap_->getSizeInCellsX(),
    std::vector<bool>(costmap_->getSizeInCellsY(), false)
  );
  std::vector<std::vector<std::shared_ptr<GraphNode>>> came_from(
    costmap_->getSizeInCellsX(),
    std::vector<std::shared_ptr<GraphNode>>(costmap_->getSizeInCellsY(), nullptr)
  );

  start_node.cost = 0;
  pending_nodes.push(start_node);

  bool goal_found = false;
  int iterations = 0;
  const int max_iterations = costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY();

  while (!pending_nodes.empty() && rclcpp::ok() && iterations < max_iterations) {
    if (cancel_checker && cancel_checker()) {
      RCLCPP_WARN(node_->get_logger(), "Planning cancelled!");
      return path;
    }

    iterations++;
    GraphNode current = pending_nodes.top();
    pending_nodes.pop();

    // Skip if already visited
    if (visited[current.x][current.y]) {
      continue;
    }
    visited[current.x][current.y] = true;

    // Goal check
    if (current.x == goal_node.x && current.y == goal_node.y) {
      goal_found = true;
      RCLCPP_INFO(node_->get_logger(), "Goal found after %d iterations!", iterations);
      break;
    }

    // Explore neighbors
    for (const auto & dir : explore_directions) {
      int nx = current.x + dir.first;
      int ny = current.y + dir.second;

      // Check bounds
      if (nx < 0 || nx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
          ny < 0 || ny >= static_cast<int>(costmap_->getSizeInCellsY())) {
        continue;
      }

      // Skip if visited
      if (visited[nx][ny]) {
        continue;
      }

      // Check if traversable (cost < INSCRIBED_INFLATED_OBSTACLE = 253)
      unsigned char cell_cost = costmap_->getCost(nx, ny);
      if (cell_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        continue;
      }

      // Calculate new cost
      int new_cost = current.cost + 1 + static_cast<int>(cell_cost);

      GraphNode neighbor(nx, ny);
      neighbor.cost = new_cost;
      neighbor.prev = std::make_shared<GraphNode>(current);

      pending_nodes.push(neighbor);
      came_from[nx][ny] = neighbor.prev;
    }
  }

  if (!goal_found) {
    RCLCPP_WARN(node_->get_logger(), "No path found after %d iterations!", iterations);
    return path;
  }

  // Reconstruct path by backtracking
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  GraphNode current = goal_node;

  // Add goal
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = global_frame_;
  goal_pose.header.stamp = node_->now();
  goal_pose.pose = gridToWorld(current);
  goal_pose.pose.orientation = goal.pose.orientation;
  poses.push_back(goal_pose);

  // Backtrack using came_from
  while (came_from[current.x][current.y] != nullptr) {
    auto prev = came_from[current.x][current.y];
    current.x = prev->x;
    current.y = prev->y;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = node_->now();
    pose.pose = gridToWorld(current);
    pose.pose.orientation.w = 1.0;
    poses.push_back(pose);
  }

  // Reverse to get start->goal order
  std::reverse(poses.begin(), poses.end());
  path.poses = poses;

  RCLCPP_INFO(node_->get_logger(), "Path found with %zu waypoints!", path.poses.size());

  return path;
}

bool DijkstraPlanner::poseOnMap(const GraphNode & node)
{
  return node.x >= 0 && node.x < static_cast<int>(costmap_->getSizeInCellsX()) &&
         node.y >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY());
}

GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
  int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
  int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
  return GraphNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(const GraphNode & node)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = (node.x + 0.5) * costmap_->getResolution() + costmap_->getOriginX();
  pose.position.y = (node.y + 0.5) * costmap_->getResolution() + costmap_->getOriginY();
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

unsigned int DijkstraPlanner::poseToCell(const GraphNode & node)
{
  return node.y * costmap_->getSizeInCellsX() + node.x;
}

}  // namespace wheelchair_planning

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wheelchair_planning::DijkstraPlanner, nav2_core::GlobalPlanner)
