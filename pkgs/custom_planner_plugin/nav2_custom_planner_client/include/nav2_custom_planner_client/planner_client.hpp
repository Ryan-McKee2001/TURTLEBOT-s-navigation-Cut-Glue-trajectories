#include <cmath>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "custom_planner_interfaces/srv/custom_planner_interface.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp" // Include the Path message

namespace nav2_custom_planner_client
{

class PlannerClient : public rclcpp::Node
{
public:
    PlannerClient(const std::string & name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Starting planner client!");
        client_ = this->create_client<custom_planner_interfaces::srv::CustomPlannerInterface>("custom_planner_server");
    }
    
    nav_msgs::msg::Path sendRequest(const std::shared_ptr<custom_planner_interfaces::srv::CustomPlannerInterface::Request>& request)
    {
        nav_msgs::msg::Path path_msg;
        auto future_result = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, std::chrono::seconds(5)) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = future_result.get();
            if (result->success)
            {
                RCLCPP_INFO(this->get_logger(), "Received successful response from server");

                // Populate poses
                for (size_t i = 0; i < result->path.data.size(); i+=2)
                {
                    geometry_msgs::msg::PoseStamped pose_stamped;
                    pose_stamped.pose.position.x = result->path.data[i];
                    pose_stamped.pose.position.y = result->path.data[i+1];
                    path_msg.poses.push_back(pose_stamped);
                }

                // Set the header of the path message
                path_msg.header.stamp = this->now();
                path_msg.header.frame_id = "global plan"; // result->path.header.frame_id; // need to fix this
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service of time out occured");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }

        return path_msg;
    }


private:
    rclcpp::Client<custom_planner_interfaces::srv::CustomPlannerInterface>::SharedPtr client_;
};

}  // namespace nav2_custom_planner_client
