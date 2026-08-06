cmake_minimum_required(VERSION 3.0.2)
project(rtabmap_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/check_runtime.sh
  scripts/check_tf.py
  scripts/inspect_bag.sh
  scripts/inspect_yangpu_bag.sh
  scripts/run_mapping.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
