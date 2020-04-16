# This file is distributed under the MIT license.
# See the LICENSE file for details.

add_subdirectory(pbrt-parser)
set(PBRTPARSER_FOUND TRUE)
set(PBRTPARSER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pbrt-parser/pbrtParser/include)
set(PBRTPARSER_LIBRARY pbrtParser)

add_subdirectory(ptex)
set(PTEX_FOUND TRUE)
set(PTEX_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ptex/src/ptex)
set(PTEX_LIBRARY Ptex_static)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/ptex/src)

set(VSNRAY_ENABLE_CUDA OFF)
set(VSNRAY_ENABLE_VIEWER OFF)
add_subdirectory(visionaray)
