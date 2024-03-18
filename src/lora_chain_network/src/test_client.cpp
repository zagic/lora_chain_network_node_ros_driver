
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include <thread>
#include <chrono>
#include "../include/lora_msg_parser_generator.h"
#include "std_msgs/String.h"
#include "driver_lora_chain_network/loraService.h"

void loraMessageCallback(const std_msgs::String::ConstPtr& msg){
    parseReceivedMessage(msg->data);
    ROS_WARN("Receive msg: %s", msg->data.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lora_test_node");
  ros::NodeHandle node;
  std::string topic_name;
  std::string service_name;

  node.param<std::string>("topic_name", topic_name, "lora/received_msg");
  node.param<std::string>("service_name", service_name, "lora/service");


  ros::Subscriber sub = node.subscribe<std_msgs::String>(topic_name, 1, loraMessageCallback);
  

  //ros::Rate loop_rate(2);
  ros::ServiceClient client = node.serviceClient<driver_lora_chain_network::loraService>(service_name);
  
  std::this_thread::sleep_for(std::chrono::seconds(2));

  driver_lora_chain_network::loraService srv;
  srv.request.command = formSoftResetCommand();

  ROS_ERROR("send command\n");
  if (client.call(srv))
  {
    ROS_WARN("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service");

  }

  while (ros::ok())
  {
    ros::spinOnce();

  }
  return 0;
  

  
}
