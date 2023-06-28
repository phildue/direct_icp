cmake_minimum_required(VERSION 3.8)
project(direct_icp VERSION 1.0.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/tools ${CMAKE_BINARY_DIR}/)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
set( sources
    src/core/Camera.cpp
    src/core/Frame.cpp
   # src/core/Feature2D.cpp
   # src/core/Point3D.cpp
   # src/core/Trajectory.cpp
    src/core/Pose.cpp
    src/core/random.cpp
   # src/utils/log.cpp
   # src/utils/visuals.cpp
    src/evaluation/evaluation.cpp
    src/evaluation/tum.cpp
    src/direct_icp/DirectIcp.cpp
   )

set( headers
   # src/core/Trajectory.h
    src/core/Pose.h
    src/core/Camera.h
    src/core/Frame.h
   # src/core/Feature2D.h
   # src/core/Point3D.h
    src/core/macros.h
    src/core/types.h
    src/core/random.h
    #src/utils/visuals.h
    #src/utils/log.h
    src/evaluation/evaluation.h
    src/evaluation/tum.h
    src/direct_icp/DirectIcp.h)
 # Construct library from sources
add_library(${PROJECT_NAME} ${sources} ${headers})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC src/)



if(UNIX)
  target_link_libraries(${PROJECT_NAME} stdc++fs)
  target_link_libraries(${PROJECT_NAME} pthread)

endif()

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

find_package(Sophus REQUIRED)
target_link_libraries(${PROJECT_NAME} Sophus::Sophus)

# While format is not implemented in gcc we use this thirdparty
find_package(fmt)
target_link_libraries(${PROJECT_NAME} fmt::fmt)


# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

find_package(TBB REQUIRED)
target_link_libraries(${PROJECT_NAME} TBB::tbb)

add_executable(main_tum src/main_tum.cpp)

target_link_libraries(main_tum ${PROJECT_NAME})


