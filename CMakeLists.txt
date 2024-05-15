cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
project(sensor_degradation_filter)

# Find dependencies
find_package(ARGoS REQUIRED)

# Add ARGoS and Buzz information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR}) # include the */lib/argos3 directory to find the ARGoS libraries
link_libraries(${ARGOS_LDFLAGS})

add_subdirectory(src) # add source directory