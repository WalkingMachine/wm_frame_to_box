cmake_minimum_required(VERSION 2.8.3)
project(wm_frame_to_box)
SET(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++11)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        tf
        tf_conversions
        eigen_conversions
        image_transport
        )


generate_messages(
        DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS std_msgs
        geometry_msgs
        roscpp
        std_msgs
        geometry_msgs
        image_transport
        tf
        tf_conversions
        eigen_conversions
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

add_executable( ${PROJECT_NAME}
        include/wm_frame_to_box.h
        src/wm_frame_to_box.cpp
        )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
