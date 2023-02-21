aux_source_directory(. DIR_LIB_SRCS)
add_library (Planner SHARED ${DIR_LIB_SRCS})
include_directories(${EIGEN3_INCLUDE_DIRS})

set(INCLUDE_DIRS
    base
    context
    goal
    planner_date
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

install(TARGETS Planner DESTINATION ${INSTALL_DIR}/lib)

foreach(dir ${INCLUDE_DIRS})
    file(GLOB_RECURSE headers "${dir}/*.hpp")
    install(FILES ${headers} DESTINATION ${INSTALL_DIR}/include/${dir})
endforeach()
install(FILES planner_interface.hpp DESTINATION ${INSTALL_DIR}/include)
