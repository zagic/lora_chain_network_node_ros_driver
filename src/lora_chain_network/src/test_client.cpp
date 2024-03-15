
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include "iostream"
#include <chrono>
#include "../include/lora_msg_parser_generator.h"

void loraMessageCallback(const ReceivedMesssageObj_t* msg ){
    ROS_INFO("Receive msg");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lora_test_node");
  ros::NodeHandle node;
  std::string topic_name;
  std::string service_name;

  node.param<std::string>("topic_name", topic_name, "lora/received_msg");
  node.param<std::string>("service_name", service_name, "lora/service");


  ros::Subscriber sub = node.subscribe<ReceivedMesssageObj_t>(topic_name, 1, loraMessageCallback);
  

  //ros::Rate loop_rate(2);
  ros::ServiceClient client = node.serviceClient<driver_lora_chain_network::loraService>(service_name);
  
  std::this_thread::sleep_for(std::chrono::seconds(2));

  driver_lora_chain_network::loraService srv;
  srv.request.cmd = formSoftResetCommand();


  if (client.call(srv))
  {
    ROS_INFO("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");

  }

  while (ros::ok())
  {
    ros::spinOnce();

  }
  return 0;
  

  
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) 