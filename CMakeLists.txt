cmake_minimum_required(VERSION 3.5.1)
project(driver_lora_chain_network)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        message_generation
        std_msgs
        serial
)

add_library(lora_msg_parser_generator
  src/lora_msg_parser_generator.cpp
)

target_include_directories(lora_msg_parser_generator
  PUBLIC ${PROJECT_SOURCE_DIR}/include
)

add_service_files(
  DIRECTORY srv
  FILES
  loraService.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


add_dependencies(lora_msg_parser_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lora_msg_parser_generator
  ${catkin_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lora_msg_parser_generator
  CATKIN_DEPENDS roscpp std_msgs message_generation message_runtime serial
)

add_executable(driver_lora_chain_network_node src/lora_chain_network_driver_node.cpp)
target_link_libraries(driver_lora_chain_network_node
  lora_msg_parser_generator
  ${catkin_LIBRARIES}
)
target_include_directories(driver_lora_chain_network_node
  PRIVATE
  ${catkin_INCLUDE_DIRS}
)


add_executable(lora_test_node src/test_client.cpp)
target_link_libraries(lora_test_node
  lora_msg_parser_generator
  ${catkin_LIBRARIES}
)
target_include_directories(lora_test_node
  PRIVATE 
  ${catkin_INCLUDE_DIRS}
)
