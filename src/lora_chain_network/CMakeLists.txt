cmake_minimum_required(VERSION 3.5.1)
project(driver_lora_chain_network)

set(CMAKE_CXX_STANDARD 14)


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
        message_generation
        )

add_service_files(
  FILES
  loraService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
        CATKIN_DEPENDS 
        roscpp 
        rospy
        message_runtime 
        std_srvs
)

add_executable(lora_chain_network_driver_node src/lora_chain_network_driver_node.cpp)
target_include_directories(lora_chain_network_driver_node
        PRIVATE
        ${catkin_INCLUDE_DIRS})

add_executable(test_client src/test_client.cpp)
target_include_directories(test_client
        PRIVATE
        ${catkin_INCLUDE_DIRS})





