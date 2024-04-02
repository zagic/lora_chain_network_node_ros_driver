#include "../include/lora_msg_parser_generator.h"
#include "driver_lora_chain_network/loraService.h"
#include <serial/serial.h>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include <thread>
#include <algorithm> 
#include <chrono>
#include "std_msgs/String.h"
#include <mutex>
#include <condition_variable>
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

std::mutex mtx;
std::condition_variable cv;

LoraReturnedStatus waitRes(int res_num,int expire_time);
void broadcastToTopic(const std::string& msg);

void emptyRxBuffer(){
  std::string line;
  while(ser_.available()){
    line = ser_.readline(100+ MAX_MESSAGE_PAYLOAD_LEN*2, "\n");
    ROS_INFO("UART received: %s", line.c_str());
  }
}


void sendCommnad(std::string command){
  ser_.write(command);
}

bool handle_service_request(driver_lora_chain_network::loraService::Request &req,
                            driver_lora_chain_network::loraService::Response &res) {
   // std::unique_lock<std::mutex> lck(mtx);
    if(taskTX !=taskStatus::IDLE){ /*when task is not free*/
      res.result=static_cast<uint8_t>(LoraReturnedStatus::RES_BUZY);
      return true;
    }
    
    ROS_INFO("Request received: %s", req.command.c_str());
    commandTosend = req.command;
    //taskTX = taskStatus::SENDING;

   // cv.wait(lck, []{return taskTX == taskStatus::RETURNING;});

    if(commandTosend.find(lora_chain_network_const::AT_CMD_RESET) != std::string::npos){
      commandType = static_cast<int>(LoraCommandType::RESET);
      sendCommnad(commandTosend);
      waitRes(1,1);
    }else if(commandTosend.find(lora_chain_network_const::AT_CMD_JOIN) != std::string::npos){
      commandType = static_cast<int>(LoraCommandType::JOIN);
      sendCommnad(commandTosend);
      waitRes(1,1);
    }else if(commandTosend.find(lora_chain_network_const::AT_CMD_STATUS) != std::string::npos){
      commandType = static_cast<int>(LoraCommandType::CHECK_STATUS);
      sendCommnad(commandTosend);
      waitRes(1,1);
    }else if(commandTosend.find(lora_chain_network_const::AT_CMD_SEND) != std::string::npos){
      commandType = static_cast<int>(LoraCommandType::SEND);
      sendCommnad(commandTosend);
      waitRes(2,2); //need two +RES_OK and ack maybe
    }else if(commandTosend.find(lora_chain_network_const::AT_CMD_CHANINFO) != std::string::npos){
      commandType = static_cast<int>(LoraCommandType::CHAN_INFO);
      sendCommnad(commandTosend);
      waitRes(3,1);  //need two +RES_OK and the chan info
    }else if(commandTosend.find(lora_chain_network_const::AT_CMD_SET_ID) != std::string::npos){
      commandType = static_cast<int>(LoraCommandType::SET_ID);
      sendCommnad(commandTosend);
      waitRes(2,1); //need two +RES_OK
    }else{
      commandType = static_cast<int>(LoraCommandType::UNKNOWN);
    }
      
    
    res.result = commandResult; 
    res.commandtype = commandType; 
    res.response_message = returnedInfo;
    return true;
}

void broadcastToTopic(const std::string& msg) {
  std_msgs::String ros_msg;
  ros_msg.data = msg;
  loraDataPublisher.publish(ros_msg);
}

void readOneLine(){
  std::string line;
  line = ser_.readline(100+ MAX_MESSAGE_PAYLOAD_LEN*2, "\n");
  ROS_INFO("UART received: %s", line.c_str());
  if(line.find(lora_chain_network_const::AT_RESPONSE_RECEIVE) != std::string::npos){
      broadcastToTopic(line);
  }else{

  }
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
        if(line.find(lora_chain_network_const::AT_RESPONSE_RES) != std::string::npos){
          if(line.find(lora_chain_network_const::AT_RETURN_RES_OK) != std::string::npos){
            num_counter++;
            if(num_counter ==res_num){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_OK);
              return LoraReturnedStatus::RES_OK;
            }
          }else if(line.find(lora_chain_network_const::AT_RETURN_RES_FORMAT_ERR) != std::string::npos){  //"FORMAT_ERROR" check needs to be ahead to "ERROR" check
              commandResult = static_cast<int>(LoraReturnedStatus::RES_COMMAND_FORMAT_ERROR);
              return LoraReturnedStatus::RES_COMMAND_FORMAT_ERROR;
          }else if(line.find(lora_chain_network_const::AT_RETURN_RES_BUZY) != std::string::npos){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_BUZY);
              return LoraReturnedStatus::RES_BUZY;
          }else if(line.find(lora_chain_network_const::AT_RETURN_RES_ERROR) != std::string::npos){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_ERROR);
              return LoraReturnedStatus::RES_ERROR;
          }else if(line.find(lora_chain_network_const::AT_RETURN_RES_TIMEOUT) != std::string::npos){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_TIMEOUT);
              return LoraReturnedStatus::RES_TIMEOUT;
          }else if(line.find(",") != std::string::npos){
              returnedInfo =line;
              num_counter++;
              if(num_counter ==res_num){
                commandResult = static_cast<int>(LoraReturnedStatus::RES_OK);
                return LoraReturnedStatus::RES_OK;
              }
          }else{
              commandResult = static_cast<int>(LoraReturnedStatus::RES_UNKNOWN);
              return LoraReturnedStatus::RES_UNKNOWN;
          }
        }else if(line.find(lora_chain_network_const::AT_RESPONSE_STATUS) != std::string::npos){
            returnedInfo =line;
            if(commandType ==static_cast<int>(LoraCommandType::CHECK_STATUS)){
              commandResult = static_cast<int>(LoraReturnedStatus::RES_OK);
              return LoraReturnedStatus::RES_OK;
            }
        }else if(line.find(lora_chain_network_const::AT_RESPONSE_RECEIVE) != std::string::npos){
            broadcastToTopic(line);
        }else{

        }
      }

  }
  commandResult = static_cast<int>(LoraReturnedStatus::RES_TIMEOUT);
  return LoraReturnedStatus::RES_TIMEOUT;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "driver_lora_chain_network_node");
  ros::NodeHandle node;
  std::string device_name;
  std::string topic_name;
  std::string service_name;
  node.param<std::string>("device_name", device_name, "/dev/ttyUSB0");
  node.param<std::string>("topic_name", topic_name, "lora/received_msg");
  node.param<std::string>("service_name", service_name, "lora/service");


  loraDataPublisher = node.advertise<std_msgs::String>(topic_name, 1000);
  loraCommandService = node.advertiseService(service_name,handle_service_request);


  //ros::Rate loop_rate(2);



  try{
      // open serial port
      ser_.setPort(device_name);
      ser_.setBaudrate(SERIAL_BAUDRATE);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser_.setTimeout(to);
      ser_.open();

      ROS_INFO("serial started");
  }
  catch (serial::IOException& e){
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }catch (std::runtime_error &error) {
    // TODO: Reset IMU
    ROS_ERROR("%s", error.what());
    return 0;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    
    if(ser_.available()){
      readOneLine();
    }
    //ROS_INFO("here3");
  }
  return 0;
  

  
}