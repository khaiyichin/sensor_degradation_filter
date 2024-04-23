
# Add loop functions library
add_library(sensor_deg_loop_functions
    SHARED
    StaticDegLoopFunctions.cpp
)

target_link_libraries(sensor_deg_loop_functions
    PRIVATE
    collective_perception
    kheperaiv_static_deg
)

target_include_directories(sensor_deg_loop_functions
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
)