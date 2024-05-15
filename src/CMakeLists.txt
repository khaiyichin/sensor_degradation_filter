add_subdirectory(algorithms)
add_subdirectory(controllers)
add_subdirectory(loop_functions)

# Add executable to start simulation
add_executable(run_dynamic_topo_simulations run_dynamic_topo_simulations.cpp)

target_link_libraries(run_dynamic_topo_simulations
    PRIVATE
    argos3core_simulator
)

# Add executable to compute wall positions
add_executable(compute_wall_positions compute_wall_positions.cpp)