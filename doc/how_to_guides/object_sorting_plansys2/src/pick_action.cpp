/*
Maintainer: Chuang Yan
Created: 2024.07.15
Description: A class for pick and place action client used in plansys2
Email: yanchuang1122@gmail.com
*/

#include <math.h>
#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gripper_action_interfaces/action/gripper_control.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
class GripperControlAction : public plansys2::ActionExecutorClient
{
    public:
        GripperControlAction()
        : plansys2::ActionExecutorClient("pick", 500ms)
        {
            using namespace std::placeholders;

            // we initialize predefined gripper states here
            // predefined_gripper_state_["open"] = 0.04;
            // predefined_gripper_state_["close"] = 0.0;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            send_feedback(0.0, "GripperControlAction running");
            gripper_control_action_client_ = rclcpp_action::create_client<gripper_action_interfaces::action::GripperControl>(
                shared_from_this(), 
                "gripper_control"
                );
            
            while(!gripper_control_action_client_->wait_for_action_server(std::chrono::seconds(1)))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the action server. Exiting.");
                    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
                }

                RCLCPP_INFO(get_logger(), "Action server not available, waiting again...");
            }


            //we use 0.0 distance to represent the state of gripper: close
            double gripper_state = 0.0;
            
            // auto args = get_arguments();
            // std::string gripper_position = args[2];
            // if(predefined_gripper_state_.find(gripper_position) != predefined_gripper_state_.end())
            // {
            //     gripper_state = predefined_gripper_state_[gripper_position];
            // } else {
            //     RCLCPP_ERROR(get_logger(), "Invalid gripper position : %s", gripper_position.c_str());
            //     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;

            // }

            auto goal_msg = gripper_action_interfaces::action::GripperControl::Goal();
            goal_msg.distance = gripper_state;

            auto send_goal_options = rclcpp_action::Client<gripper_action_interfaces::action::GripperControl>::SendGoalOptions();
            send_goal_options.feedback_callback = 
                std::bind(&GripperControlAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&GripperControlAction::result_callback, this, std::placeholders::_1);

            future_goal_handle_ = gripper_control_action_client_->async_send_goal(goal_msg, send_goal_options);

            return ActionExecutorClient::on_activate(previous_state);
        }

    private:
        void feedback_callback
            (
                rclcpp_action::ClientGoalHandle<gripper_action_interfaces::action::GripperControl>::SharedPtr goal_handle,
                const std::shared_ptr<const gripper_action_interfaces::action::GripperControl::Feedback> feedback
            )
        {
            send_feedback(0.5, "GripperControlAction running");
        };

        void result_callback
            (
                const rclcpp_action::ClientGoalHandle<gripper_action_interfaces::action::GripperControl>::WrappedResult & result
            )
        {
            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    finish(true, 1.0, "GripperControlAction completed");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    finish(false, 0.0, "GripperControlAction aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    finish(false, 0.0, "GripperControlAction canceled");
                    break;
                default:
                    finish(false, 0.0, "GripperControlAction failed");
                    break;
            }

        };

        void do_work()
        {
        };

        std::map<std::string, double> predefined_gripper_state_;
        rclcpp_action::Client<gripper_action_interfaces::action::GripperControl>::SharedPtr gripper_control_action_client_;
        std::shared_future<rclcpp_action::ClientGoalHandle<gripper_action_interfaces::action::GripperControl>::SharedPtr> future_goal_handle_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperControlAction>();

    node->set_parameter(rclcpp::Parameter("action_name", "pick")); 
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
    
}