#include "driver_lora_chain_network.h"
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include "iostream"
#include <chrono>
#include "lora_msg_parser_generator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lora_test_node");
  ros::NodeHandle node;
  std::string topic_name;
  std::string service_name;

  node.param<std::string>("topic_name", topic_name, "lora/received_msg");
  node.param<std::string>("service_name", service_name, "lora/service");


  ros::Subscriber sub = nh.subscribe<ReceivedMesssageObj_t>(topic_name, 1, loraMessageCallback);
  

  //ros::Rate loop_rate(2);
  ros::ServiceClient client = n.serviceClient<driver_lora_chain_network::loraService>(service_name);
  
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


    return 0;

    // 创建Subscriber，订阅指定topic的PointCloud2消息，指定回调函数
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic_name, 1, pointCloudCallback);
