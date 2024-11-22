
# Add controller library
add_library(${PROJECT_NAME}_controllers
    SHARED
    KheperaIVDiffusionMotion.cpp
)

target_link_libraries(${PROJECT_NAME}_controllers
    PRIVATE
    ${PROJECT_NAME}_algorithms
)

target_include_directories(${PROJECT_NAME}_controllers
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
)