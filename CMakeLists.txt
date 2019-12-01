cmake_minimum_required(VERSION 2.8.3)
project(terprescue)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(OpenCV_PATH "/usr/local")
set(OpenCV_INCLUDE_DIRS "${OpenCV_PATH}/include")
set(OpenCV_LIBS "${OpenCV_PATH}/lib")

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    tf
    cv_bridge
)
find_package(OpenCV 3.4.8 EXACT REQUIRED PATHS ${OpenCV_PATH})

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

message("\n\nOPENCV VERSION:\n" ${OpenCV_INCLUDE_DIRS} "\n!!!!\n\n")

add_executable(main src/main.cpp src/localizer.cpp src/terprescue.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    #add_rostest_gtest(serviceTest launch/serviceTest.test test/test.cpp)
    #target_link_libraries(serviceTest ${catkin_LIBRARIES})
endif()
