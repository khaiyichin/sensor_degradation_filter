# Add collective perception library
add_library(${PROJECT_NAME}_algorithms
    SHARED
    CollectivePerception.cpp
    StaticDegradationFilterAlpha.cpp
)

target_include_directories(${PROJECT_NAME}_algorithms
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
)