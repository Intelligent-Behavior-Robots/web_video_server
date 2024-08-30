cmake_minimum_required(VERSION 3.8)
project(video_client_pkg)

add_compile_options(-std=c++11)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
                  libavutil
                  libavcodec
                  libavformat
                  libswscale
                  libavfilter)

find_package(CURL REQUIRED)


include_directories(
  ${CURL_INCLUDE_DIRS}
)

add_executable(video_client_node src/main.cpp)


target_link_libraries(video_client_node
  PkgConfig::LIBAV
  ${CURL_LIBRARIES}
)


install(TARGETS video_client_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
