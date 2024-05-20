
# Add loop functions library
add_library(${PROJECT_NAME}_loop_functions
    SHARED
    ../Arena.cpp
    StaticDegLoopFunctions.cpp
)

target_link_libraries(${PROJECT_NAME}_loop_functions
    PRIVATE
    ${PROJECT_NAME}_algorithms
    ${PROJECT_NAME}_controllers
)

target_include_directories(${PROJECT_NAME}_loop_functions
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
)