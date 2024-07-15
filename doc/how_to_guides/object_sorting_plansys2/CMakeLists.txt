cmake_minimum_required(VERSION 3.8)
project(object_sorting_plansys2)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(moveit_action_interfaces REQUIRED)
find_package(gripper_action_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    moveit_action_interfaces
    gripper_action_interfaces
)

add_executable(move_to_pose_action src/move_to_pose_action.cpp)
ament_target_dependencies(move_to_pose_action ${dependencies})

add_executable(pick_action src/pick_action.cpp)
ament_target_dependencies(pick_action ${dependencies})

add_executable(place_action src/place_action.cpp)
ament_target_dependencies(place_action ${dependencies})

add_executable(object_sorting_controller_node src/object_sorting_controller_node.cpp)
ament_target_dependencies(object_sorting_controller_node ${dependencies})

install(TARGETS 
  move_to_pose_action
  pick_action
  place_action
  object_sorting_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
