cmake_minimum_required(VERSION 2.8.3)

project(LaserScanDetector)

set(ROS_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS genmsg message_generation roscpp)

add_message_files(FILES
				  Object.msg
				  ObjectDetection.msg
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE LaserScanDetector_src "src/*.cpp")

add_executable(LaserScanDetector ${LaserScanDetector_src})
target_link_libraries(LaserScanDetector ${catkin_LIBRARIES})

add_dependencies(LaserScanDetector ${PROJECT_NAME}_generate_messages_cpp)
