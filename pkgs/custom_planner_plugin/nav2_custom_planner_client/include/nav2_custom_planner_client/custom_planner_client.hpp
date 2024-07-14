#ifndef NAV2_CUSTOM_PLANNER_CLIENT__CUSTOM_PLANNER_CLIENT_HPP_
#define NAV2_CUSTOM_PLANNER_CLIENT__CUSTOM_PLANNER_CLIENT_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_custom_planner_client/planner_client.hpp"

namespace nav2_custom_planner_client
{

class CustomPlannerClient : public nav2_core::GlobalPlanner
{
public:
  CustomPlannerClient() = default;
  ~CustomPlannerClient() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  double interpolation_resolution_;

  // Planner client
  std::shared_ptr<PlannerClient> planner_client_;
};

}  // namespace nav2_custom_planner_client

#endif  // NAV2_CUSTOM_PLANNER_CLIENT__CUSTOM_PLANNER_CLIENT_HPP_
