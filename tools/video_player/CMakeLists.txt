cmake_minimum_required(VERSION 3.16)
project(self_guiding_dart_video_player LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(video_player main.cpp)
target_compile_features(video_player PRIVATE cxx_std_17)
target_compile_options(video_player PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(video_player PRIVATE Qt5::Widgets)

install(TARGETS video_player RUNTIME DESTINATION bin)
