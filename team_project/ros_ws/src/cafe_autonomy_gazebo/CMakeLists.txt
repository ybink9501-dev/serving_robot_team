cmake_minimum_required(VERSION 3.8)

project(cafe_autonomy_gazebo)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
add_executable(gazebo_truth_evidence src/truth_relay.cpp)
ament_target_dependencies(gazebo_truth_evidence rclcpp tf2_msgs)
target_link_libraries(gazebo_truth_evidence gz-transport13::core gz-msgs10::gz-msgs10)
install(TARGETS gazebo_truth_evidence DESTINATION lib/${PROJECT_NAME})

find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
add_library(team_contact_snapshot SHARED src/contact_snapshot.cpp)
target_link_libraries(team_contact_snapshot gz-sim8::gz-sim8 gz-transport13::core gz-msgs10::gz-msgs10 gz-plugin2::register)
install(TARGETS team_contact_snapshot LIBRARY DESTINATION lib)

find_package(rosgraph_msgs REQUIRED)
add_executable(team_sim_clock src/clock_relay.cpp)
ament_target_dependencies(team_sim_clock rclcpp rosgraph_msgs)
target_link_libraries(team_sim_clock gz-transport13::core gz-msgs10::gz-msgs10)
install(TARGETS team_sim_clock DESTINATION lib/${PROJECT_NAME})

ament_package()
