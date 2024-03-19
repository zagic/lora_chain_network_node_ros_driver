
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include <thread>
#include <chrono>
#include "../include/lora_msg_parser_generator.h"
#include "std_msgs/String.h"
#include "driver_lora_chain_network/loraService.h"

#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cstdio>

void loraMessageCallback(const std_msgs::String::ConstPtr& msg){
    ReceivedMesssageObj_t obj = parseReceivedMessage(msg->data);
    if(obj.type == MessageType::DATA_TRAFFIC ){
      std::string data = toHexString(obj.payload, obj.len);
      ROS_WARN("Receive msg: %s from %02X%02X", data.c_str(), obj.Source[0], obj.Source[1] );
    }else if(obj.type == MessageType::ACKNOWLEDGEMENT){
      int session_id = obj.payload[0] + obj.payload[1]*256;
      ROS_WARN("Receive ACK: %d from %02X%02X", session_id ,obj.Source[0], obj.Source[1] );
    }
    ROS_WARN("Receive msg: %s", msg->data.c_str());
}

void setStdinNonBlocking() {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    flags |= O_NONBLOCK;
    fcntl(STDIN_FILENO, F_SETFL, flags);
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
  
  //std::this_thread::sleep_for(std::chrono::seconds(2));
  //setStdinNonBlocking();

  driver_lora_chain_network::loraService srv;


  std::string input;
  std::string instruction = "Support tests: \n Type \"0,XXXX\" set ID\n Type \"1\" join\n Type \"2\" reset\n Type \"3,XXXX,q,AABBCCDDD\" send hex data in bytes with q(qos) = 0 or 2 or 4 or6 \n Type \"4\" request chan_info\n Type \"5\" check status\n";

  LoraCommandType cmd_type;
  CommandResult_t result_object;
  std::cout << instruction << std::endl;
  while (ros::ok())
  {
    ros::spinOnce();
    if(std::cin >> input) {
        ROS_WARN("receive command %s",input.c_str());
        int cmd_idx;
        try{
          cmd_idx= std::stoi(input.substr(0,1));
        
        
          cmd_type = static_cast<LoraCommandType>(cmd_idx);
          switch(cmd_type){
            case LoraCommandType::SET_ID:
            {
              std::vector<uint8_t> tmp = fromHexString(input, DEVICE_ID_LEN ,  2 );
              if(tmp.size() != DEVICE_ID_LEN){
                ROS_WARN("ID not right");
                break;
              }else{
                srv.request.command = formSetIDCommand(tmp.data());
                if (client.call(srv))
                {
                  if(static_cast<LoraReturnedStatus>(srv.response.result) ==LoraReturnedStatus::RES_OK){
                    ROS_WARN("Set ID OK");
                  }else{
                    ROS_WARN("Set ID failed");
                  }
                }
                else
                {
                  ROS_ERROR("Failed to call service");
                }
              }
            }
            break;
            case LoraCommandType::JOIN:
              srv.request.command = formJoinCommand();
              if (client.call(srv))
              {
                if(static_cast<LoraReturnedStatus>(srv.response.result) ==LoraReturnedStatus::RES_OK){
                  ROS_WARN("Start to Join");
                }else{
                  ROS_WARN("Join failed");
                }
              }
              else
              {
                ROS_ERROR("Failed to call service");
              }
            break;
            case LoraCommandType::RESET:
              srv.request.command = formSoftResetCommand();
              if (client.call(srv))
              {
                if(static_cast<LoraReturnedStatus>(srv.response.result) ==LoraReturnedStatus::RES_OK){
                  ROS_WARN("Reset OK");
                }else{
                  ROS_WARN("Reset failed");
                }
              }
              else
              {
                ROS_ERROR("Failed to call service");
              }
            break;
            case LoraCommandType::SEND:
            {
              std::vector<uint8_t> tmp = fromHexString(input, DEVICE_ID_LEN ,  2 );
              if(tmp.size() != DEVICE_ID_LEN){
                ROS_WARN("ID not right");
                break;
              }
              size_t expect_size = std::min( (input.size()-9)/2, static_cast<size_t>(MAX_MESSAGE_PAYLOAD_LEN ));;
              std::vector<uint8_t> buf = fromHexString(input, expect_size  ,  9 );
              int qos;
              try{
                  qos = std::stoi(input.substr(7,1));

              }catch(...){
                ROS_WARN("Qos not right");
                  break;
              }
              
              if(buf.size() != expect_size){
                ROS_WARN("Message not right %d,%d",buf.size() ,expect_size);
                break;
              }
              srv.request.command = formSendCommand( tmp.data(), static_cast<QosLevel>(qos), buf.data(), expect_size);
              if (client.call(srv))
              {
                result_object = parseCommandResult(srv.response.commandtype,srv.response.result,srv.response.response_message);
                if(result_object.status ==LoraReturnedStatus::RES_OK && result_object.cmd==LoraCommandType::SEND ){
                  ROS_WARN("Message sent\n");
                  if(qos>=4){
                    ROS_WARN("SessionbID %d\n",result_object.data.session_id);
                  }
                }else{
                  ROS_WARN("Message TX failed");
                }
              }
              else
              {
                ROS_ERROR("Failed to call service");
              }
            }
            break;
            case LoraCommandType::CHAN_INFO:
              srv.request.command = formChanInfoCommand();
              if (client.call(srv))
              {
                result_object = parseCommandResult(srv.response.commandtype,srv.response.result,srv.response.response_message);
                if(result_object.status ==LoraReturnedStatus::RES_OK && result_object.cmd==LoraCommandType::CHAN_INFO ){
                  ROS_WARN("Get chan info: %d\n",result_object.data.chanInfoList.len);
                  for(int i =0;i<result_object.data.chanInfoList.len;i++){
                    ROS_WARN("ID %02X%02X, idx %d, RSS %d, SNR %d",
                    result_object.data.chanInfoList.list[i].id[0],result_object.data.chanInfoList.list[i].id[1],
                    result_object.data.chanInfoList.list[i].idx,
                    result_object.data.chanInfoList.list[i].rss,
                    result_object.data.chanInfoList.list[i].snr);
                  }
                }else{
                  ROS_WARN("Chan info request failed");
                }
              }
              else
              {
                ROS_ERROR("Failed to call service");
              }
            break;
            case LoraCommandType::CHECK_STATUS:
              srv.request.command = formStatusCheckCommand();
              if (client.call(srv))
              {
                result_object = parseCommandResult(srv.response.commandtype,srv.response.result,srv.response.response_message);
                if(result_object.status ==LoraReturnedStatus::RES_OK && result_object.cmd==LoraCommandType::CHECK_STATUS){
                  switch(result_object.data.node_status){
                    case LoraNodeStatus::NODE_IDLE:
                    ROS_WARN("Status IDLE");
                    break;
                    case LoraNodeStatus::NODE_BUZY:
                    ROS_WARN("Status BUZY");
                    break;
                    case LoraNodeStatus::NODE_TXRX:
                    ROS_WARN("Status TXRX");
                    break;
                    case LoraNodeStatus::NODE_SEARCHING:
                    ROS_WARN("Status Searching");
                    break;
                    default:
                    ROS_WARN("Status Unknown");
                    break;
                  }
                }else{
                  ROS_WARN("Status check failed");
                }
              }
              else
              {
                ROS_ERROR("Failed to call service");
              }
            break;
            default:
              ROS_WARN("Unsupport command");
            break;
          }
        }catch(...){
          ROS_INFO("unrecognized input,try again");
          
        }

    }
  }
  return 0;
  

  
}
