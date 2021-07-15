cmake_minimum_required(VERSION 3.19)
project(gpd)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# #include "omp.h" / omp_get_wtime()
set(CMAKE_CXX_FLAGS "-fopenmp")

include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Generate the shared library from the sources
add_library(${PROJECT_NAME}_cloud SHARED src/${PROJECT_NAME}/util/cloud.cpp src/${PROJECT_NAME}/util/eigen_utils.cpp)
target_link_libraries(${PROJECT_NAME}_cloud ${PCL_LIBRARIES})

add_executable (${PROJECT_NAME}_test_gpd test_gpd.cpp)
target_link_libraries (${PROJECT_NAME}_test_gpd ${PCL_LIBRARIES} ${PROJECT_NAME}_cloud)

add_executable (${PROJECT_NAME}_test_other test_other.cpp)
target_link_libraries (${PROJECT_NAME}_test_other ${PCL_LIBRARIES} ${PROJECT_NAME}_cloud)