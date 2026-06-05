cmake_minimum_required(VERSION 3.20)

project(RoadSignTrafficLightDetection
    VERSION 1.0.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Fallback when OpenCV_DIR is not set (e.g. CMake Tools F7 build).
if(DEFINED ENV{OpenCV_DIR} AND NOT OpenCV_DIR)
    set(OpenCV_DIR "$ENV{OpenCV_DIR}" CACHE PATH "OpenCV CMake package directory")
elseif(NOT OpenCV_DIR AND EXISTS "C:/opencv/build/OpenCVConfig.cmake")
    set(OpenCV_DIR "C:/opencv/build" CACHE PATH "OpenCV CMake package directory")
endif()

find_package(OpenCV 4 REQUIRED)

add_library(RoadSignCore STATIC
    src/DebugImageWriter.cpp
    src/DetectionSelection.cpp
    src/DisplayOnTerminal.cpp
    src/HsvMaskSegmenter.cpp
    src/RoadObjectDetector.cpp
    src/TextReader.cpp
    src/TrafficLightFinder.cpp
    src/ShapeAnalyzer.cpp
    src/ColorSegmenter.cpp
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

# Copy OpenCV runtime DLLs next to the executable so Shift+F5/debug works without PATH tweaks.
if(WIN32)
    set(OPENCV_BIN_DIR "${OpenCV_DIR}/x64/vc16/bin")
    file(GLOB OPENCV_RUNTIME_DLLS
        "${OPENCV_BIN_DIR}/opencv_world*.dll"
        "${OPENCV_BIN_DIR}/opencv_videoio_ffmpeg*.dll"
    )
    foreach(OPENCV_RUNTIME_DLL IN LISTS OPENCV_RUNTIME_DLLS)
        add_custom_command(TARGET RoadSignDetector POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${OPENCV_RUNTIME_DLL}"
                $<TARGET_FILE_DIR:RoadSignDetector>
            COMMENT "Copying ${OPENCV_RUNTIME_DLL}"
        )
    endforeach()
endif()
