#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "custom_planner_interfaces/srv/custom_planner_interface.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_custom_planner_client/custom_planner_client.hpp"

#include <iostream>
#include <vector>

namespace nav2_custom_planner_client
{

// Implement the methods of the CustomPlannerClient class

void CustomPlannerClient::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Create and initialize the planner client
  planner_client_ = std::make_shared<PlannerClient>("planner_client");

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void CustomPlannerClient::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void CustomPlannerClient::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void CustomPlannerClient::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path CustomPlannerClient::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal)
{
    nav_msgs::msg::Path global_path;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_)
    {
        RCLCPP_ERROR(
            node_->get_logger(), "Planner will only accept start position from %s frame",
            global_frame_.c_str());
        return global_path;
    }

    if (goal.header.frame_id != global_frame_)
    {
        RCLCPP_INFO(
            node_->get_logger(), "Planner will only accept goal position from %s frame",
            global_frame_.c_str());
        return global_path;
    }

    // Construct the service request
    auto request = std::make_shared<custom_planner_interfaces::srv::CustomPlannerInterface::Request>();
    request->start_pose = start;
    request->end_pose = goal;
    // Construct the occupancy grid
    request->occupancy_grid.header.stamp = node_->now();
    request->occupancy_grid.header.frame_id = global_frame_;
    request->occupancy_grid.info.resolution = costmap_->getResolution();
    request->occupancy_grid.info.width = costmap_->getSizeInCellsX();
    request->occupancy_grid.info.height = costmap_->getSizeInCellsY();
    request->occupancy_grid.info.origin.position.x = costmap_->getOriginX();
    request->occupancy_grid.info.origin.position.y = costmap_->getOriginY();
    request->occupancy_grid.info.origin.position.z = 0.0;
    request->occupancy_grid.info.origin.orientation.x = 0.0;
    request->occupancy_grid.info.origin.orientation.y = 0.0;
    request->occupancy_grid.info.origin.orientation.z = 0.0;
    request->occupancy_grid.info.origin.orientation.w = 1.0;
    request->occupancy_grid.data.resize(request->occupancy_grid.info.width * request->occupancy_grid.info.height);

    // Append occupancy grid data with costmap data which will be sent to client
    for (unsigned int y = 0; y < request->occupancy_grid.info.height; ++y)
    {
        for (unsigned int x = 0; x < request->occupancy_grid.info.width; ++x)
        {
            unsigned int cost = costmap_->getCost(x, y);
            request->occupancy_grid.data[y * request->occupancy_grid.info.width + x] = cost;
        }
    }

    // Call the sendRequest function and store the result
    global_path = planner_client_->sendRequest(request);

    // Post-process the path to include the goal pose
    if (!global_path.poses.empty())
    {
        geometry_msgs::msg::PoseStamped goal_pose = goal;
        goal_pose.header.stamp = node_->now();
        goal_pose.header.frame_id = global_frame_;
        global_path.poses.push_back(goal_pose);
    }

    // Set header information for the returned path
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    return global_path;
}

}  // namespace nav2_custom_planner_client

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner_client::CustomPlannerClient, nav2_core::GlobalPlanner)
