file(GLOB_RECURSE SOURCE_FILES "*.cpp" "base/*.cpp" 
    "context/*.cpp" "goal/*.cpp" "planner_data/*.cpp"
    "rrt_simple/*.cpp" "tools/checker/*.cpp" "tools/distance/*.cpp"
    "tools/local_planner/*.cpp" "tools/nearest_neighbor/*.cpp"
    "tools/path_simplifier/*.cpp" "tools/sampler/*.cpp"
    "utility/*.cpp")
# aux_source_directory(. DIR_LIB_SRCS)
message("SOURCE_FILES: "${SOURCE_FILES})
add_library (Shaohe_Planner STATIC ${SOURCE_FILES})
include_directories(${EIGEN3_INCLUDE_DIRS})

set(INCLUDE_DIRS
    base
    context
    goal
    planner_data
    rrt_simple
    tools/checker
    tools/distance
    tools/local_planner
    tools/nearest_neighbor
    tools/path_simplifier
    tools/sampler
    utility
)
include_directories(${INCLUDE_DIRS})
set(INSTALL_DIR ~/ws_moveit2/install/planner)

install(TARGETS Shaohe_Planner DESTINATION ${INSTALL_DIR}/lib)

foreach(dir ${INCLUDE_DIRS})
    file(GLOB_RECURSE headers "${dir}/*.hpp")
    install(FILES ${headers} DESTINATION ${INSTALL_DIR}/include/${dir})
endforeach()
install(FILES planner_interface.hpp DESTINATION ${INSTALL_DIR}/include)
