cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(pcl_filteration)
# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find the PCL package
find_package(PCL 1.10 REQUIRED COMPONENTS)

# Get the current directory path
# get_filename_component(MY_CURRENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" )

# Include the PCL headers
include_directories(${PCL_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add the Pass Through Filter executable
# add_executable(PassThroughfilter src/PassThroughfilter.cpp)
# add_executable(bladePointcloud src/bladePointcloud.cpp)
# add_executable(filteredPointCloud src/filteredPointCloud.cpp)
# add_executable(filterBladeWithOrigin src/filterBladeWithOrigin.cpp)
# add_executable(drawPlane src/drawPlane.cpp)
# add_executable(magnitudeWDirection src/magnitudeWDirection.cpp)
add_executable(clustring src/clustring.cpp)
add_executable(don src/don.cpp)
add_executable(readPointCloud src/readPointCloud.cpp)
add_executable(planeFitting src/planeFitting.cpp)
add_executable(main src/main.cpp)
add_executable(writeCsv src/writeCsv.cpp)

# add_executable(bladAngleDetection src/bladAngleDetection.cpp)

# Link against the PCL libraries
# target_link_libraries(PassThroughfilter ${PCL_LIBRARIES})
# target_link_libraries(bladePointcloud ${PCL_LIBRARIES})
# target_link_libraries(filteredPointCloud ${PCL_LIBRARIES})
# target_link_libraries(filterBladeWithOrigin ${PCL_LIBRARIES})
# target_link_libraries(drawPlane ${PCL_LIBRARIES})
# target_link_libraries(magnitudeWDirection ${PCL_LIBRARIES})
target_link_libraries(clustring ${PCL_LIBRARIES})
target_link_libraries(don ${PCL_LIBRARIES})
target_link_libraries(readPointCloud ${PCL_LIBRARIES})
target_link_libraries(planeFitting ${PCL_LIBRARIES})
target_link_libraries(main ${PCL_LIBRARIES})
target_link_libraries(writeCsv ${PCL_LIBRARIES})

# target_link_libraries(bladAngleDetection ${PCL_LIBRARIES})

# Print the current directory variable
# message("Current directory: ${MY_CURRENT_DIR}")