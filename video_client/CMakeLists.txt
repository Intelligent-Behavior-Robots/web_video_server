cmake_minimum_required(VERSION 3.10)
project(VideoClient)

include_directories(${CMAKE_SOURCE_DIR}/third_parties/ffmpeg/arm64-v8a/include)

add_executable(${PROJECT_NAME} main.cpp)

target_link_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/third_parties/ffmpeg/arm64-v8a/lib)

target_link_libraries(${PROJECT_NAME}
avcodec
avdevice
avfilter
avformat
avutil
swresample
swscale
)