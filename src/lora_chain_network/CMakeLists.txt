cmake_minimum_required(VERSION 3.5.1)
project(driver_lora_chain_network)

set(CMAKE_CXX_STANDARD 14)

add_library(lora_chain_network_driver_lib
        src/driver_lora_chain_network.cpp
        src/serial_unix.cpp)

target_include_directories(lora_chain_network_driver_lib
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )
target_link_libraries(lora_chain_network_driver_lib)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
	std_srvs
        tf2
        tf2_ros
        )
catkin_package(
        CATKIN_DEPENDS 
        sensor_msgs 
        std_srvs
        tf2
        tf2_ros
)

add_executable(lora_chain_network_driver_node src/lora_chain_network_driver_node.cpp)
target_include_directories(lora_chain_network_driver_node
        PRIVATE
        ${catkin_INCLUDE_DIRS})
target_link_libraries(lora_chain_network_driver_node PRIVATE lora_chain_network_driver_lib ${catkin_LIBRARIES})





