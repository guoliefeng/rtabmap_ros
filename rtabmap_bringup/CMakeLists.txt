cmake_minimum_required(VERSION 3.0.2)
project(rtabmap_bringup)

find_package(catkin REQUIRED COMPONENTS pcl_ros)
find_package(RTABMap REQUIRED COMPONENTS core)
find_package(PCL 1.7 REQUIRED COMPONENTS common filters io)

catkin_package(CATKIN_DEPENDS pcl_ros)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(rtabmap_db_to_pcd src/rtabmap_db_to_pcd.cpp)
target_link_libraries(rtabmap_db_to_pcd
  rtabmap::core
  ${PCL_LIBRARIES}
)

add_executable(pointcloud_safety_filter src/pointcloud_safety_filter.cpp)
target_link_libraries(pointcloud_safety_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/check_runtime.sh
  scripts/check_fastlio_rtabmap_runtime.sh
  scripts/check_fastlio_sync.py
  scripts/analyze_rtabmap_db.py
  scripts/export_rtabmap_pcd.sh
  scripts/inspect_pcd.py
  scripts/fastlio_odom_covariance.py
  scripts/gnss_odom_to_global_pose.py
  scripts/run_fastlio_frontend_mode_b.sh
  scripts/run_fastlio_rtabmap_bag_test.sh
  scripts/run_fastlio_rtabmap_multisession_test.sh
  scripts/check_tf.py
  scripts/inspect_bag.sh
  scripts/inspect_yangpu_bag.sh
  scripts/run_mapping.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rtabmap_db_to_pcd pointcloud_safety_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
