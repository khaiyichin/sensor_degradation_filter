# Add collective perception library
add_library(${PROJECT_NAME}_algorithms
    SHARED
    CollectivePerception.cpp
    StaticDegradationFilterAlpha.cpp
    StaticDegradationFilterBravo.cpp
    DynamicDegradationFilterCharlie.cpp
    ELBO.cpp
)

target_include_directories(${PROJECT_NAME}_algorithms
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}_algorithms
    PUBLIC
    GSL::gsl
    NLopt::nlopt
)