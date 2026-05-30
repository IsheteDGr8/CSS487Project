cmake_minimum_required(VERSION 3.20)

project(RoadSignTrafficLightDetection
    VERSION 1.0.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED)

add_library(RoadSignCore STATIC
    src/DebugImageWriter.cpp
    src/DisplayOnTerminal.cpp
    src/HsvMaskSegmenter.cpp
    src/RoadObjectDetector.cpp
)

target_include_directories(RoadSignCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(RoadSignCore
    PUBLIC
        ${OpenCV_LIBS}
)

add_executable(RoadSignDetector
    src/main.cpp
)

target_link_libraries(RoadSignDetector
    PRIVATE
        RoadSignCore
)

if(MSVC)
    target_compile_options(RoadSignCore PRIVATE /W4 /permissive-)
    target_compile_options(RoadSignDetector PRIVATE /W4 /permissive-)
else()
    target_compile_options(RoadSignCore PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(RoadSignDetector PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)

if(BUILD_TESTING)
    add_executable(DetectorSmokeTest
        tests/detector_smoke_test.cpp
    )

    target_link_libraries(DetectorSmokeTest
        PRIVATE
            RoadSignCore
    )

    if(MSVC)
        target_compile_options(DetectorSmokeTest PRIVATE /W4 /permissive-)
    else()
        target_compile_options(DetectorSmokeTest PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME DetectorSmokeTest COMMAND DetectorSmokeTest)
endif()
