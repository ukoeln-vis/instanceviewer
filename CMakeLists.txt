project(InstanceViewer)

find_package(GLEW REQUIRED)

cmake_minimum_required(VERSION 2.8.9)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "${PROJECT_SOURCE_DIR}/cmake"
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(3rdparty)

set(VSNRAY_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/3rdparty/visionaray/config")
include_directories(${VSNRAY_CONFIG_DIR})

include_directories(3rdparty/pbrt-parser/pbrtParser/include)
include_directories(3rdparty/visionaray/include)
include_directories(3rdparty/visionaray/src)

add_executable(InstanceViewer InstanceViewer.cpp)

target_link_libraries(InstanceViewer pbrtParser)
target_link_libraries(InstanceViewer visionaray)
target_link_libraries(InstanceViewer visionaray_common)
target_link_libraries(InstanceViewer ${GLEW_LIBRARY})
