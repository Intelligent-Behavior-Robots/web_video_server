cmake_minimum_required(VERSION 3.10)
project(VideoClient)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
                  libavutil
                  libavcodec
                  libavformat
                  libswscale
                  libavfilter)

find_package(CURL REQUIRED)

include_directories(${CURL_INCLUDE_DIRS})

add_executable(video_client main.cpp)
target_link_libraries(video_client
                      PkgConfig::LIBAV
                      ${CURL_LIBRARIES})
