cmake_minimum_required(VERSION 2.8)
project(brife_main)

find_package(OpenCV 3.1)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(brife_main brife_main.cpp)
target_link_libraries(brife_main ${OpenCV_LIBS})

add_executable(chess_pad chess_pad.cpp)
target_link_libraries(chess_pad ${OpenCV_LIBS})

add_executable(create_black create_black.cpp)
target_link_libraries(create_black ${OpenCV_LIBS})
