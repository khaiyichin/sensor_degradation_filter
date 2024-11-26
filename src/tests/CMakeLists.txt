# Add collective perception library
add_executable(test_elbo_class test_elbo_class.cpp)

target_include_directories(test_elbo_class
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
)

target_link_libraries(test_elbo_class
    PRIVATE
    ${PROJECT_NAME}_algorithms
)