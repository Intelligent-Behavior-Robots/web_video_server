cmake_minimum_required(VERSION 3.10)
project(VideoClient)



include_directories(${CMAKE_SOURCE_DIR}/third_parties/include)


add_executable(${PROJECT_NAME} main.cpp)



target_link_libraries(${PROJECT_NAME}
                       ${CMAKE_SOURCE_DIR}/third_parties/lib/libavcodec.so
                       ${CMAKE_SOURCE_DIR}/third_parties/lib/libavdevice.so
                       ${CMAKE_SOURCE_DIR}/third_parties/lib/libavfilter.so
                       ${CMAKE_SOURCE_DIR}/third_parties/lib/libavformat.so
                       ${CMAKE_SOURCE_DIR}/third_parties/lib/libavutil.so
                       ${CMAKE_SOURCE_DIR}/third_parties/lib/libswresample.so
                       ${CMAKE_SOURCE_DIR}/third_parties/lib/libswscale.so
                       )     
                       
         

add_executable(video_client main.cpp)

