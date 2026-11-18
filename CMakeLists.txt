cmake_minimum_required(VERSION 3.1.3)
project(cr5_car_moveit)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(crop_floor src/crop_floor.cpp)
target_link_libraries(crop_floor ${catkin_LIBRARIES} octomap)
