cmake_minimum_required(VERSION 2.8)
project(icp_lidar_ceres)

set(ICP_LIDAR_CERES_VERSION "1.0.0")

if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    set(COMPILING_WITH_CATKIN 1)
endif()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

find_package(PythonLibs REQUIRED)
find_package(Ceres REQUIRED)
if(COMPILING_WITH_CATKIN)
    message(STATUS "---------------")
    message(STATUS "Catkin detected")
    message(STATUS "---------------")
    find_package(catkin REQUIRED)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
    )
endif()

include_directories(
    include
    "/usr/include/eigen3"
    "/usr/local/include/knn"
    ${PYTHON_INCLUDE_DIRS}
    "/usr/local/lib/python3.6/dist-packages/numpy/core/include"
)

if(COMPILING_WITH_CATKIN)
    add_library(${PROJECT_NAME} src/icp_lidar_ceres.cpp)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
    target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES} ${PYTHON_LIBRARIES})
    install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
    install(DIRECTORY ../include/icp_lidar_ceres/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/icp_lidar_ceres
        PATTERN ".svn" EXCLUDE
    )
else()
    add_executable(${PROJECT_NAME} src/icp_lidar_ceres.cpp src/icp_lidar_ceres_main.cpp)
    target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES} ${PYTHON_LIBRARIES})
endif()