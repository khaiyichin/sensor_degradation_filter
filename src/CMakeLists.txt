add_subdirectory(controllers)
add_subdirectory(loop_functions)

# Add collective perception library
add_library(collective_perception
    SHARED
    CollectivePerceptionAlgorithm.cpp
    Arena.cpp
)

target_include_directories(collective_perception
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
)

# Add executable to start simulation
add_executable(run_dynamic_topo_simulations run_dynamic_topo_simulations.cpp)

target_link_libraries(run_dynamic_topo_simulations
    PRIVATE
    argos3core_simulator
)

# Add executable to compute wall positions
add_executable(compute_wall_positions compute_wall_positions.cpp)