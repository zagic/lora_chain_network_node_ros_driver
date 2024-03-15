#include "../include/lora_msg_parser_generator.h"
#include "driver_lora_chain_network/loraService.h"
#include <serial/serial.h>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include "iostream"
#include <algorithm> 
#include <chrono>

serial::Serial ser_;


ros::Publisher loraDataPublisher;
ros::ServiceServer loraCommandService;

/* variables to achieve asynchronized lora command TX pocess*/
enum class taskStatus{
  IDLE,
  SENDING,
  RETURNING,
};

volatile taskStatus taskTX = taskStatus::IDLE;
std:: string commandTosend ="";
volatile int commandType = -1;
volatile int commandResult = -1;
std:: string returnedInfo ="";

constexpr int SERIAL_BAUDRATE =115200;



void emptyRxBuffer(){
  while(ser_.available()){
  }
}

bool handle_service_request(driver_lora_chain_network::loraService::Request &req,
                            driver_lora_chain_network::loraService::Response &res) {
    if(taskTX !=taskStatus::IDLE){ /*when task is not free*/
      res.command=static_cast<uint8_t>(LoraReturnedStatus::RES_BUZY);
      return true;
    }
    ROS_INFO("Request received: %s", req.command.c_str());
    commandTosend = req.command;
    taskTX = taskStatus::SENDING;
    while(taskTX != taskStatus::RETURNING){
      ros::Duration(0.01).sleep();
    }

    res.result = commandResult; 
    res.commandtype = commandType; 
    res.response_message = returnedInfo;
    
    return true;
}

void broadcastToTopic(std::string msg){
  ReceivedMesssageObj_t tr;
  tr.len =0;
  std::istringstream iss(msg);
  std::string token,id,repeater_id,qos,data;
  if(std::getline(iss, token, ' ')){   /*There is a space after +receive= */
    if(std::getline(iss, id, ',')){  
        if(std::getline(iss, repeater_id, ',')){  
          if(std::getline(iss, qos, ',')){  
            if(std::getline(iss, data, ' ')){  
              std::vector<uint8_t> id_vector = fromHexString(id,DEVICE_ID_LEN,0);
              if(id_vector.size()==DEVICE_ID_LEN){
                  for(int i =0;i<DEVICE_ID_LEN;i++){
                      tr.Source[i] = id_vector[i];
                  }
              }else{
                  break;
              }

              std::vector<uint8_t> data_vector = fromHexString(data,data.size()/2,0);
              if(data_vector.size() ==data.size()/2){
                tr.len = std:min(data_vector.size(),MAX_MESSAGE_PAYLOAD_LEN);
                memcpy(tr.payload,data_vector.data(),tr.len);
              }else{
                break;
              }

              std::vector<uint8_t> qos_vector = fromHexString(qos,1,0);
              uint8_t qos_u8 =  qos_vector& 0xF0;
              tr.MessageType = static_cast<MessageType>(qos_u8);
              loraDataPublisher.publish(tr);
              return;

            }
          }
        }
    }
  }
  ROS_WARN("Lora message parse failed");
  return;
}

/* some command expect two +res=OK and some just need one, 
 chanInfo require another +res which contains the channel info */
LoraReturnedStatus waitRes(int res_num,int expire_time){
  int num_counter = 0;
  std::string line;
  std::chrono::seconds timeout(expire_time); 
  auto start = std::chrono::steady_clock::now(); 

  while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start) < timeout) {

      if(ser_.available()){
        
        line = ser_.readline(100+ MAX_MESSAGE_PAYLOAD_LEN*2, "\n");
        ROS_INFO("UART received: %s", line.c_str());
        if(line.find(AT_RESPONSE_RES) != std::string::npos){
          if(line.find(AT_RETURN_RES_OK) != std::string::npos){
            num_counter++;
            if(num_counter ==res_num){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_OK);
              return commandResult;
            }
          }else if(line.find(AT_RETURN_RES_BUZY) != std::string::npos){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_BUZY);
              return commandResult;
          }else if(line.find(AT_RETURN_RES_ERROR) != std::string::npos){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_ERROR);
              return commandResult;
          }else if(line.find(AT_RETURN_RES_TIMEOUT) != std::string::npos){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_TIMEOUT);
              return commandResult;
          }else if(line.find(AT_RETURN_RES_FORMAT_ERR) != std::string::npos){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_COMMAND_FORMAT_ERROR);
              return commandResult;
          }else if(line.find(",") != std::string::npos){
              returnedInfo =line;
              num_counter++;
              if(num_counter ==res_num){
                commandResult = static_cast<int>(LoraReturnedStatus::RES_OK);
                return commandResult;
              }
          }else{
              commandResult = static_cast<int>(LoraReturnedStatus::RES_UNKNOWN);
              return commandResult;
          }
        }else if(line.find(AT_RESPONSE_STATUS) != std::string::npos){
            returnedInfo =line;
        }else if(line.find(AT_RESPONSE_RECEIVE) != std::string::npos){
            broadcastToTopic(line);
        }else{

        }
      }

  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lora_chain_network_driver_node");
  ros::NodeHandle node;
  std::string device_name;
  std::string topic_name;
  std::string service_name;
  node.param<std::string>("device_name", device_name, "/dev/ttyUSB0");
  node.param<std::string>("topic_name", topic_name, "lora/received_msg");
  node.param<std::string>("service_name", service_name, "lora/service");


  loraDataPublisher = node.advertise<ReceivedMesssageObj_t>(topic_name, 1000);
  loraCommandService = node.advertiseService(service_name,handle_service_request);


  //ros::Rate loop_rate(2);



  try{
      // 打开串口
      ser_.setPort(device_name);
      ser_.setBaudrate(SERIAL_BAUDRATE);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser_.setTimeout(to);
      ser_.open();

      
  }
  catch (serial::IOException& e){
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }catch (std::runtime_error &error) {
    // TODO: Reset IMU
    ROS_ERROR("%s\n", error.what());
    return 0;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    if(taskTX == taskStatus::RETURNING){
      if(commandTosend.find(AT_CMD_RESET) != std::string::npos){
        commandType = static_cast<int>(LoraCommandType::RESET);

      }
    }
    
    if(ser_.available()){
    }
  }
  return 0;
  

  
}