/*
Project:     object_sorting_plansys2
Description: The controller node is to describe a use case for object sorting using plansys2.
Maintainer:  Chuang Yan
Created:     2024.07.15
Email:       yanchuang1122@gmail.com
*/

#include <plansys2_pddl_parser/Utils.h>
#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ObjectSortingController : public rclcpp::Node
{
public:
    ObjectSortingController()
    : rclcpp::Node("object_sorting_controller"), state_(STARTING)
    {
    }
    

    void init()
    {
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        init_knowledge();
    }

    void init_knowledge()
    {
        problem_expert_->addInstance(plansys2::Instance{"franka", "manipulator"});
        problem_expert_->addInstance(plansys2::Instance{"pose1_i", "pose"});
        problem_expert_->addInstance(plansys2::Instance{"pose1_d", "pose"});
        problem_expert_->addInstance(plansys2::Instance{"pose2_i", "pose"});
        problem_expert_->addInstance(plansys2::Instance{"pose2_d", "pose"});
        problem_expert_->addInstance(plansys2::Instance{"red_cube", "object"});
        problem_expert_->addInstance(plansys2::Instance{"blue_cube", "object"});

        problem_expert_->addPredicate(plansys2::Predicate("(manipulator_at franka pose1_i)"));
        problem_expert_->addPredicate(plansys2::Predicate("(object_at red_cube pose1_i)"));
        problem_expert_->addPredicate(plansys2::Predicate("(object_at blue_cube pose2_i)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected pose1_i pose1_d)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected pose1_d pose1_i)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected pose1_d pose2_i)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected pose2_i pose1_d)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected pose2_i pose2_d)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected pose2_d pose2_i)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected pose2_d pose1_i)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected pose1_i pose2_d)"));


    }
    void step()
    {
        switch (state_)
        {
            case STARTING:
                {
                    // Set the goal for next state
                    problem_expert_->setGoal(plansys2::Goal("(and(object_at red_cube pose1_d))"));

                    // Compute the plan
                    auto domain = domain_expert_->getDomain();
                    auto problem = problem_expert_->getProblem();
                    auto plan = planner_client_->getPlan(domain, problem);

                    if(!plan.has_value())
                    {
                        std::cout << "Could not find plan to reach goal" <<
                            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                        break;
                    }

                    // execute the plan
                    if (executor_client_->start_plan_execution(plan.value()))
                    {
                        state_ = OBJECTAT_P1D;
                    }

                }
                break;

            case OBJECTAT_P1D:
                {
                    auto feedback = executor_client_->getFeedBack();
                    
                    for (const auto & action_feedback : feedback.action_execution_status)   
                    {
                        std::cout << "[" << action_feedback.action << " " <<
                            action_feedback.completion * 100.0 << "%]";
                    }
                    std::cout << std::endl;
                    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
                    {
                        if(executor_client_->getResult().value().success)
                        {
                            std::cout << "Successful finished " << std::endl;

                            // Cleanning up
                            problem_expert_->removePredicate(plansys2::Predicate("(and(object_at red_cube pose1_d))"));

                            // set the goal for next state
                            problem_expert_->setGoal(plansys2::Goal("(and(object_at blue_cube pose2_d))"));

                            // Compute the plan
                            auto domain = domain_expert_->getDomain();
                            auto problem = problem_expert_->getProblem();
                            auto plan = planner_client_->getPlan(domain, problem);

                            if (!plan.has_value())
                            {
                                std::cout << "Could not find plan to reach goal " <<
                                    parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                                break;
                            }

                            // execute the plan
                            if (executor_client_->start_plan_execution(plan.value()))
                            {
                                state_ = STOP;
                            }
                        } else
                        {
                            for (const auto & action_feedback : feedback.action_execution_status)
                            {
                                if(action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED)
                                {
                                    std::cout << "[" << action_feedback.action << "] finished with error: " <<
                                        action_feedback.message_status << std::endl;
                                }
                            }

                            // Replan
                            auto domain = domain_expert_->getDomain();
                            auto problem = problem_expert_->getProblem();
                            auto plan = planner_client_->getPlan(domain, problem);

                            if (!plan.has_value()) {
                                std::cout << "Unsuccessful replan attempt to reach goal " <<
                                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                                break;
                            }

                            // Execute the plan
                            executor_client_->start_plan_execution(plan.value());
                        }
                    }
                }
                break;

            case STOP:
                {
                }
                break;

            default:
                break;
        }
    }

private:
    typedef enum {STARTING, OBJECTAT_P1D, STOP} StateType;
    StateType state_;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectSortingController>();

    node->init();
    rclcpp::Rate rate(5);
    while (rclcpp::ok())
    {
        node->step();
        rate.sleep();
        rclcpp::spin_some(node->get_node_base_interface());
    }

    rclcpp::shutdown();

    return 0;

}