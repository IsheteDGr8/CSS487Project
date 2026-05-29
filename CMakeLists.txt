cmake_minimum_required(VERSION 3.20)

project(RoadSignTrafficLightDetection
    VERSION 1.0.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED)

add_executable(RoadSignDetector
    src/main.cpp
)

target_include_directories(RoadSignDetector
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(RoadSignDetector
    PRIVATE
        ${OpenCV_LIBS}
)

if(MSVC)
    target_compile_options(RoadSignDetector PRIVATE /W4 /permissive-)
else()
    target_compile_options(RoadSignDetector PRIVATE -Wall -Wextra -Wpedantic)
endif()
