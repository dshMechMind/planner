cmake_minimum_required(VERSION 3.5)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wall -Wextra -Wpedantic")
add_compile_options("-O3")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Debug")

project(Planner)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_subdirectory(src)
# set(INCLUDE_DIRS
#     src/base
#     src/context
#     src/goal
#     src/planner_date
#     src/rrt_simple
#     src/tools/checker
#     src/tools/distance
#     src/tools/local_planner
#     src/tools/nearest_neighbor
#     src/tools/path_simplifier
#     src/tools/sampler
#     src/utility
#     src/planner_interface
# )
# include_directories(${INCLUDE_DIRS})
# set(INSTALL_DIR ~/ws_moveit2/install/planner)

# aux_source_directory(src SOURCES)
# add_library (Planner SHARED ${SOURCES})
# target_include_directories(Planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# message("install lib....")
# message(${SOURCES})
# install(TARGETS Planner DESTINATION ${INSTALL_DIR}/lib)

# message("install include....")
# foreach(dir ${INCLUDE_DIRS})
#     file(GLOB_RECURSE headers "${dir}/*.hpp")
#     install(FILES ${headers} DESTINATION ${INSTALL_DIR}/include/${dir})
# endforeach()
# install(FILES src/planner_interface.hpp DESTINATION ${INSTALL_DIR}/include/)
# file(GLOB_RECURSE includes *.hpp)
# install(FILES ${includes} DESTINATION ~/ws_moveit2/install/planner/include)
