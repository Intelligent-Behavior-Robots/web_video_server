cmake_minimum_required(VERSION 3.8)
project(video_client_pkg)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
                  libavutil
                  libavcodec
                  libavformat
                  libswscale
                  libavfilter)



# Create executable
add_executable(video_client_node src/main.cpp)


ament_target_dependencies(video_client_node
  rclcpp
  std_msgs
  sensor_msgs
)


target_link_libraries(video_client_node
  PkgConfig::LIBAV
)

# Install the node
install(TARGETS video_client_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
