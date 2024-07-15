/*
Maintainer: Chuang Yan
Created: 2024.07.12
Description: A class for moveit action client used in plansys2
Email: yanchuang1122@gmail.com
*/

#include <math.h>
#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_action_interfaces/action/move_to_pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveToPoseAction : public plansys2::ActionExecutorClient
{
public:
    MoveToPoseAction()
    : plansys2::ActionExecutorClient("move_it", 500ms)
    {
        using namespace std::placeholders;

        // we initialize predefined poses here
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        // !!! ===== need to double check the frame_id ===== !!!
        pose.header.frame_id = "panda_link0";
        pose.pose.position.x = 0.3;
        pose.pose.position.y = 0.3;
        pose.pose.position.z = 0.1;
        pose.pose.orientation.x = -0.3827;
        pose.pose.orientation.y = 0.9239;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.0;
        predefined_poses_["pose1_i"] = pose; // initial pose for pose1(first cube)

        pose.pose.position.x = 0.3;
        pose.pose.position.y = -0.3;
        pose.pose.position.z = 0.1;
        predefined_poses_["pose1_d"] = pose; // desired pose for pose1(first cube)

        pose.pose.position.x = 0.4;
        pose.pose.position.y = 0.3;
        pose.pose.position.z = 0.1;
        predefined_poses_["pose2_i"] = pose; // initial pose for pose2(second cube) and so on...

        pose.pose.position.x = 0.4;
        pose.pose.position.y = -0.3;
        pose.pose.position.z = 0.1;
        predefined_poses_["pose2_d"] = pose;

        // Future: add function to read predefined poses from yaml file or json file

    } // end of MoveToPoseAction constructor


    // on_activate function to be called when the action is activated from the lifecycle manager
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        send_feedback(0.0, "MoveToPose action activated");

        // create a client for the move_to_pose action
        move_to_pose_action_client_ = rclcpp_action::create_client<moveit_action_interfaces::action::MoveToPose>(
            shared_from_this(),
            "move_to_pose"
        );

        // wait for the action server to be available
        while(!move_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the action server. ");
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            RCLCPP_INFO(get_logger(), "Waiting for the action server to be up...");
        }

        // get the target pose from action arguments
        auto args = get_arguments();
        std::string target_pose = args[2]; // the third argument is the target pose !!! defubed by plansys2

        // check if the target pose is in the predefined poses
        if(predefined_poses_.find(target_pose) == predefined_poses_.end())
        {
            RCLCPP_ERROR(get_logger(), "Target pose not found in predefined poses");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // send the goal to the action server
        auto goal_msg = moveit_action_interfaces::action::MoveToPose::Goal();
        goal_msg.target_pose = predefined_poses_[target_pose].pose;

        auto send_goal_options = rclcpp_action::Client<moveit_action_interfaces::action::MoveToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&MoveToPoseAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MoveToPoseAction::result_callback, this, std::placeholders::_1);

        future_goal_handle_ = move_to_pose_action_client_->async_send_goal(goal_msg, send_goal_options);

        return ActionExecutorClient::on_activate(previous_state);
    }


private:

    // getDistance function to calculate the distance between two poses
    double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
    {
        return sqrt(
            pow(pos1.position.x - pos2.position.x, 2) +
            pow(pos1.position.y - pos2.position.y, 2) +
            pow(pos1.position.z - pos2.position.z, 2)
        );
    }

    // feedback_callback
    void feedback_callback(
        rclcpp_action::ClientGoalHandle<moveit_action_interfaces::action::MoveToPose>::SharedPtr,
        const std::shared_ptr<const moveit_action_interfaces::action::MoveToPose::Feedback> feedback)
    {
        send_feedback(feedback->completion_percentage, "MoveToPose running");
    }

    // result_callback
    void result_callback
    (
        const rclcpp_action::ClientGoalHandle<moveit_action_interfaces::action::MoveToPose>::WrappedResult & result
    )
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            finish(true, 1.0, "MoveToPose completed");
            break;
        
        case rclcpp_action::ResultCode::ABORTED:
            finish(false, 0.0, "MoveToPose aborted");
            break;

        case rclcpp_action::ResultCode::CANCELED:
            finish(false, 0.0, "MoveToPose canceled");
            break;
        
        default:
            finish(false, 0.0, "MoveToPose failed");
            break;
        }
    }

    // from plansys2, needed if we don't have a server running.
    void do_work()
    {

    }

    std::map<std::string, geometry_msgs::msg::PoseStamped> predefined_poses_;
    rclcpp_action::Client<moveit_action_interfaces::action::MoveToPose>::SharedPtr move_to_pose_action_client_;
    std::shared_future<rclcpp_action::ClientGoalHandle<moveit_action_interfaces::action::MoveToPose>::SharedPtr> future_goal_handle_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPoseAction>();

    node->set_parameter(rclcpp::Parameter("action_name", "move_it"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}