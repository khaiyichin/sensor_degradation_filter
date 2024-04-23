
# Add controller library
add_library(kheperaiv_static_deg
    SHARED
    KheperaIVStaticDeg.cpp
)

target_link_libraries(kheperaiv_static_deg
    PRIVATE
    collective_perception
)

target_include_directories(kheperaiv_static_deg
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
)