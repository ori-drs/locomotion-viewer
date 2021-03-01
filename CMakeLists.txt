cmake_minimum_required(VERSION 3.0.2)
project(locomotion_viewer)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  roscpp
  rviz_visual_tools
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz_visual_tools geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/LocomotionViewer.cpp
        src/ZeroDim.cpp
        src/OneDim.cpp
        src/TwoDim.cpp
        src/ThreeDim.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install libraries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
