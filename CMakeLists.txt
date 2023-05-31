cmake_minimum_required(VERSION 3.12)
project(PassThroughfilter)
project(bladePointcloud)
project(filteredPointCloud)
project(filterBladeWithOrigin)
project(drawPlane)
project(magnitudeWDirection)
project(clastring)
project(readPointCloud)
project(planeFitting)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find the PCL package
find_package(PCL 1.2 REQUIRED)

# Include the PCL headers
include_directories(${PCL_INCLUDE_DIRS})

# Add the Pass Through Filter executable
add_executable(PassThroughfilter PassThroughfilter.cpp)
add_executable(bladePointcloud bladePointcloud.cpp)
add_executable(filteredPointCloud filteredPointCloud.cpp)
add_executable(filterBladeWithOrigin filterBladeWithOrigin.cpp)
add_executable(drawPlane drawPlane.cpp)
add_executable(magnitudeWDirection magnitudeWDirection.cpp)
add_executable(clustring clustring.cpp)
add_executable(readPointCloud readPointCloud.cpp)
add_executable(planeFitting planeFitting.cpp)

# Link against the PCL libraries
target_link_libraries(PassThroughfilter ${PCL_LIBRARIES})
target_link_libraries(bladePointcloud ${PCL_LIBRARIES})
target_link_libraries(filteredPointCloud ${PCL_LIBRARIES})
target_link_libraries(filterBladeWithOrigin ${PCL_LIBRARIES})
target_link_libraries(drawPlane ${PCL_LIBRARIES})
target_link_libraries(magnitudeWDirection ${PCL_LIBRARIES})
target_link_libraries(clustring ${PCL_LIBRARIES})
target_link_libraries(readPointCloud ${PCL_LIBRARIES})
target_link_libraries(planeFitting ${PCL_LIBRARIES})