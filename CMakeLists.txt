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
        cv_bridge
        sara_msgs
        )

#add_message_files(
#        DIRECTORY msg
#        FILES
#)

add_service_files(
        DIRECTORY srv
        FILES
        GetBoundingBoxes3D.srv
)

generate_messages(
        DEPENDENCIES
        sara_msgs
        geometry_msgs
        std_msgs
        sensor_msgs
)

catkin_package(
        CATKIN_DEPENDS std_msgs
        geometry_msgs
        roscpp
        std_msgs
        geometry_msgs
        image_transport
        tf
        tf_conversions
        eigen_conversions
        sara_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable( ${PROJECT_NAME}
        src/wm_frame_to_box.cpp
        )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} wm_frame_to_box_generate_messages)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
