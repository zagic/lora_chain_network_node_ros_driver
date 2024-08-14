
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
#include <array>

void loraMessageCallback(const std_msgs::String::ConstPtr& msg){
    if(msg->data.find(lora_chain_network_const::AT_RESPONSE_RECEIVE) != std::string::npos){
      ReceivedMesssageObj_t obj = parseReceivedMessage(msg->data);
      if(obj.type == MessageType::DATA_TRAFFIC ){
        std::string data = toHexString(obj.payload, obj.len);
        ROS_INFO("Receive msg: %s from %02X%02X", data.c_str(), obj.Source[0], obj.Source[1] );
      }else if(obj.type == MessageType::ACKNOWLEDGEMENT){
        int session_id = obj.payload[0] + obj.payload[1]*256;
        ROS_INFO("Receive ACK: %d from %02X%02X", session_id ,obj.Source[0], obj.Source[1] );
      }else{
        ROS_INFO("Receive unrecognized msg: %s" , msg->data.c_str());
      }
    
    }else if(msg->data.find(lora_chain_network_const::AT_RESPONSE_EVENT) != std::string::npos){
      if(msg->data.find(lora_chain_network_const::AT_EVENT_DISCONNECT) != std::string::npos){
        ROS_INFO("REPEATER disconnected" );
      }else if(msg->data.find(lora_chain_network_const::AT_EVENT_CONNECTED) != std::string::npos){
        ROS_INFO("REPEATER connected" );
      }
    }
    
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
  setStdinNonBlocking();

  driver_lora_chain_network::loraService srv;


  std::string input;
  std::string instruction = "Support tests: \n Type \"0,XXXX\"             set ID\n Type \"1\"                  join\n Type \"2\"                  reset\n Type \"3,XXXX,q,AABBCCDDD\" send hex data in bytes with q(qos) = 0 or 2 or 4 or6 \n Type \"4\"                  request chan_info\n Type \"5\"                  check status\n Type \"6\"                  rejoin\n Type \"7\"                  quit network\n";

  LoraCommandType cmd_type;
  CommandResult_t result_object;
  std::array<char, 1000> buffer;
  ssize_t bytesRead;

  std::cout << instruction << std::endl;
  while (ros::ok())
  {
    ros::spinOnce();
    bytesRead = read(STDIN_FILENO, buffer.data(), buffer.size() - 1);
    if (bytesRead > 0) {
        buffer[bytesRead] = '\0'; // Ensure null-termination
        input = std::string(buffer.data(), bytesRead);
    
        int cmd_idx;
        try{
          cmd_idx= std::stoi(input.substr(0,1));
        
        
          cmd_type = static_cast<LoraCommandType>(cmd_idx);
          switch(cmd_type){
            case LoraCommandType::SET_ID:
            {
              if(input.size()<(DEVICE_ID_LEN*2+2)){
                ROS_INFO("ID not right");
                break;
              }
              std::vector<uint8_t> tmp = fromHexString(input, DEVICE_ID_LEN ,  2 );
              if(tmp.size() != DEVICE_ID_LEN){
                ROS_INFO("ID not right");
                break;
              }else{
                srv.request.command = formSetIDCommand(tmp.data());
                if (client.call(srv))
                {
                  if(static_cast<LoraReturnedStatus>(srv.response.result) ==LoraReturnedStatus::RES_OK){
                    ROS_INFO("Set ID OK");
                  }else{
                    ROS_INFO("Set ID failed");
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
                  ROS_INFO("Start to Join");
                }else{
                  ROS_INFO("Join failed");
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
                  ROS_INFO("Reset OK");
                }else{
                  ROS_INFO("Reset failed");
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
                ROS_INFO("ID not right");
                break;
              }
              if((input.size()-9-1)%2 !=0){  // there is a "\n" at last
                ROS_INFO("Payload is not a hexstring");
                break;
              }
              size_t expect_size = std::min( (input.size()-9-1)/2, static_cast<size_t>(MAX_MESSAGE_PAYLOAD_LEN ));;
              std::vector<uint8_t> buf = fromHexString(input, expect_size  ,  9 );
              int qos;
              try{
                  qos = std::stoi(input.substr(7,1));

              }catch(...){
                ROS_INFO("Qos not right");
                  break;
              }
              
              if(buf.size() != expect_size){
                ROS_INFO("Message not right %d,%d",(int)buf.size() ,(int)expect_size);
                break;
              }
              srv.request.command = formSendCommand( tmp.data(), static_cast<QosLevel>(qos), buf.data(), expect_size);
              if (client.call(srv))
              {
                result_object = parseCommandResult(srv.response.commandtype,srv.response.result,srv.response.response_message);
                if(result_object.status ==LoraReturnedStatus::RES_OK && result_object.cmd==LoraCommandType::SEND ){
                  ROS_INFO("Message sent");
                  if(qos>=4){
                    ROS_INFO("SessionID %d",result_object.data.session_id);
                  }
                }else{
                  ROS_INFO("Message TX failed, errcode: %d",(int)result_object.status);
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
                  ROS_INFO("Get chan info: %d",result_object.data.chanInfoList.len);
                  for(int i =0;i<result_object.data.chanInfoList.len;i++){
                    ROS_INFO("ID %02X%02X, idx %d, RSS %d, SNR %d",
                    result_object.data.chanInfoList.list[i].id[0],result_object.data.chanInfoList.list[i].id[1],
                    result_object.data.chanInfoList.list[i].idx,
                    result_object.data.chanInfoList.list[i].rss,
                    result_object.data.chanInfoList.list[i].snr);
                  }
                }else{
                  ROS_INFO("Chan info request failed");
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
                    ROS_INFO("Status IDLE");
                    break;
                    case LoraNodeStatus::NODE_BUZY:
                    ROS_INFO("Status BUZY");
                    break;
                    case LoraNodeStatus::NODE_TXRX:
                    ROS_INFO("Status TXRX");
                    break;
                    case LoraNodeStatus::NODE_SEARCHING:
                    ROS_INFO("Status Searching");
                    break;
                    default:
                    ROS_INFO("Status Unknown");
                    break;
                  }
                }else{
                  ROS_INFO("Status check failed");
                }
              }
              else
              {
                ROS_ERROR("Failed to call service");
              }
            break;
            case LoraCommandType::REJOIN:
              srv.request.command = formRejoinCommand();
              if (client.call(srv))
              {
                if(static_cast<LoraReturnedStatus>(srv.response.result) ==LoraReturnedStatus::RES_OK){
                  ROS_INFO("Start to Rejoin");
                }else{
                  ROS_INFO("Rejoin failed");
                }
              }
              else
              {
                ROS_ERROR("Failed to call service");
              }
            break;
            case LoraCommandType::QUIT:
              srv.request.command = formQuitCommand();
              if (client.call(srv))
              {
                if(static_cast<LoraReturnedStatus>(srv.response.result) ==LoraReturnedStatus::RES_OK){
                  ROS_INFO("Start to Quit");
                }else{
                  ROS_INFO("Quit failed");
                }
              }
              else
              {
                ROS_ERROR("Failed to call service");
              }
            break;
            default:
              ROS_INFO("Unsupport command");
            break;
          }
        }catch(...){
          ROS_INFO("unrecognized input,try again");
          
        }

    }
    usleep(100000); 
  }
  return 0;
  

  
}
