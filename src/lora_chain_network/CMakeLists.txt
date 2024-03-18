cmake_minimum_required(VERSION 3.5.1)
project(driver_lora_chain_network)

set(CMAKE_CXX_STANDARD 14)



find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
	std_srvs
        message_generation
        serial 
        )

add_library(lora_msg_parser_generator
  src/lora_msg_parser_generator.cpp
)


add_service_files(
  FILES
  loraService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

target_link_libraries(lora_msg_parser_generator
  ${catkin_LIBRARIES}

)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES lora_msg_parser_generator
        CATKIN_DEPENDS 
        roscpp 
        rospy
        message_runtime 
        std_srvs
        std_msgs
        serial
)

add_executable(driver_lora_chain_network_node src/driver_lora_chain_network_node.cpp)
target_include_directories(driver_lora_chain_network_node
        PRIVATE
        ${catkin_LIBRARIES})

add_executable(lora_test_node src/test_client.cpp)
target_include_directories(lora_test_node
        PRIVATE
        ${catkin_LIBRARIES})





