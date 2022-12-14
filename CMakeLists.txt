cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_updater)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_msgs roscpp rostest std_msgs nav_msgs)

catkin_python_setup()

catkin_package(DEPENDS diagnostic_msgs roscpp std_msgs
               INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Caricamento dei file cpp dei nodi Updater

add_executable(cam_updater src/cam_updater.cpp)
add_executable(odometry_updater src/odometry_updater.cpp)
add_executable(laser_scan_updater src/laser_scan_updater.cpp)
add_executable(imu_updater src/imu_updater.cpp)
add_executable(joint_right_wheel src/joint_right_wheel.cpp)
add_executable(joint_left_wheel src/joint_left_wheel.cpp)
add_executable(cmd_vel_updater src/cmd_vel_updater.cpp)
add_executable(odometry_ekf_updater src/odometry_ekf_updater.cpp)
add_executable(odometry_ground_truth_odom_updater src/odometry_ground_truth_odom_updater.cpp)

# Caricamento delle librerie per ogni nodo Updater

target_link_libraries(cam_updater ${catkin_LIBRARIES})
target_link_libraries(odometry_updater ${catkin_LIBRARIES})
target_link_libraries(laser_scan_updater ${catkin_LIBRARIES})
target_link_libraries(imu_updater ${catkin_LIBRARIES})
target_link_libraries(joint_right_wheel ${catkin_LIBRARIES})
target_link_libraries(joint_left_wheel ${catkin_LIBRARIES})
target_link_libraries(cmd_vel_updater ${catkin_LIBRARIES})
target_link_libraries(odometry_ekf_updater ${catkin_LIBRARIES})
target_link_libraries(odometry_ground_truth_odom_updater ${catkin_LIBRARIES})

catkin_add_gtest(diagnostic_updater_test test/diagnostic_updater_test.cpp)
target_link_libraries(diagnostic_updater_test ${catkin_LIBRARIES})
add_rostest(test/diagnostic_updater_test.xml)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
