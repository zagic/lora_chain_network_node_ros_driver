#include "../include/lora_msg_parser_generator.h"
#include <sstream>
#include <iomanip>
#include <cstring>
#include <algorithm> 
#include <iostream>

std::string toHexString(const uint8_t* data, size_t length) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for(size_t i = 0; i < length; ++i) {
        ss << std::setw(2) << static_cast<int>(data[i]);
    }
    return ss.str();
}

bool is_hex_string_valid(const std::string& hexStr) {
    if (hexStr.length() % 2 != 0) return false; // Length must be even
    for (char c : hexStr) {
        if (!std::isxdigit(c)) return false; // Must be a hex digit
    }
    return true;
}

std::vector<uint8_t> fromHexString(const std::string& hexStr, size_t bytelen, size_t startPos) {
    std::vector<uint8_t> data;

    if (startPos >= hexStr.length() ) {
        return data; 
    }
    size_t endPoint;
    if(bytelen*2+startPos <= hexStr.length()){
        endPoint = bytelen*2+startPos;
    }else{
        endPoint = hexStr.length();
    }
    for (size_t i = startPos; i < endPoint; i += 2) {
        std::string byteString = hexStr.substr(i, 2);
        if(!is_hex_string_valid(byteString )){
            data.clear();
            return data;
        }
        uint8_t byte = static_cast<uint8_t>(std::strtol(byteString.c_str(), nullptr, 16));
        data.push_back(byte);
    }
    return data;
}





/**
 * @brief Config ID od the end-device, PAN ID is reserved as 0000 
 *
 * @param id the two-bytes-long ID 
 * @return Generated command string
 */
std::string formSetIDCommand(uint8_t * id) {
  return lora_chain_network_const::AT_CMD_SET_ID +"="+ toHexString(id, DEVICE_ID_LEN) +",0000" +"\r\n";
}


/**
 * @brief Soft reset command
 * @return Generated command string
 */
std::string formSoftResetCommand() {
    return lora_chain_network_const::AT_CMD_RESET+"\r\n";
}


/**
 * @brief Join netwokr command
 * @return Generated command string
 */
std::string formJoinCommand() {
    return lora_chain_network_const::AT_CMD_JOIN+"\r\n";
}

/**
 * @brief status check command
 * @return Generated command string
 */
std::string formStatusCheckCommand() {
    return lora_chain_network_const::AT_CMD_STATUS+"?\r\n";
}

/**
 * @brief Config ID od the end-device, PAN ID is reserved as 0000 
 *
 * @param target  ID of the destination device
 * @param qos     The priority of the message, also determine if ACK is required
 * @param payload message body, in hex format
 * @param len     payload length
 * @return Generated command string
 */
std::string formSendCommand(uint8_t * target, QosLevel qos, uint8_t * payload, int len) {
    uint8_t tmp = static_cast<uint8_t>(qos);
    return lora_chain_network_const::AT_CMD_SEND +"="+ toHexString(target, DEVICE_ID_LEN) 
    +",0,"+toHexString(payload, len)+ ","+ toHexString(&tmp, 1)+"\r\n";
}

/**
 * @brief request channel info command
 * @return Generated command string
 */
std::string formChanInfoCommand() {
    return lora_chain_network_const::AT_CMD_CHANINFO+"=0\r\n";
}

std::string formRejoinCommand(){
    return lora_chain_network_const::AT_CMD_REJOIN+"\r\n";
}
std::string formQuitCommand(){
    return lora_chain_network_const::AT_CMD_QUIT+"\r\n";
}


CommandResult_t parseCommandResult(int command, int res, std::string response){

    CommandResult_t tr;
    switch(res){
        case 0:
            tr.status = LoraReturnedStatus::RES_OK;
            break;
        case 1:
            tr.status = LoraReturnedStatus::RES_BUZY;
            break;
        case 2:
            tr.status = LoraReturnedStatus::RES_ERROR;
            break;
        case 3:
            tr.status = LoraReturnedStatus::RES_TIMEOUT;
            break;
        case 4:
            tr.status = LoraReturnedStatus::RES_UNKNOWN;
            break;
        case 5:
            tr.status = LoraReturnedStatus::RES_COMMAND_FORMAT_ERROR;
            break;
        case 6:
            tr.status = LoraReturnedStatus::RES_UART_NO_RESPONSE;
            break;
        default:
            tr.status = LoraReturnedStatus::RES_UNKNOWN;
            break;
    }

    switch(command){
        case 0:
            tr.cmd = LoraCommandType::SET_ID;
            return tr;
        case 1:
            tr.cmd = LoraCommandType::JOIN;
            return tr;
        case 2:
            tr.cmd = LoraCommandType::RESET;
            return tr;
        case 3:
            tr.cmd = LoraCommandType::SEND;
            if(response.find(lora_chain_network_const::AT_RESPONSE_STATUS) != std::string::npos){
                std::istringstream iss(response);
                std::string token,session_id;
                if(std::getline(iss,token, ',')){
                    if(std::getline(iss,token, ',')){
                        if(std::getline(iss,session_id, ',')){
                            std::vector<uint8_t> sesion_vector = fromHexString(session_id,2,0);
                            if((sesion_vector.size())==2){
                                tr.data.session_id = sesion_vector[0]+sesion_vector[1]*256;
                            }else{
                                tr.status = LoraReturnedStatus::RES_ERROR;
                            }
                            
                        }
                    }
                }
            }
            return tr;
        case 4:
        {       
            tr.cmd = LoraCommandType::CHAN_INFO;
            uint8_t num =0;
            if(response.find(lora_chain_network_const::AT_RESPONSE_RES) != std::string::npos){
                std::istringstream iss(response);
                std::string token,id,idx,rss,snr;
                if(std::getline(iss, token, ' ')){   /*There is a space after +res= */
                    while(std::getline(iss,id, ',')){
                        if(std::getline(iss,idx, ',')){
                            if(std::getline(iss,rss, ',')){
                                if(std::getline(iss,snr, ',')){
                                    std::vector<uint8_t> id_vector = fromHexString(id,DEVICE_ID_LEN,0);
                                    if(id_vector.size()==DEVICE_ID_LEN){
                                        for(int i =0;i<DEVICE_ID_LEN;i++){
                                            tr.data.chanInfoList.list[num].id[i] = id_vector[i];
                                        }
                                    }else{
                                        break;
                                    }
                                    try{
                                        tr.data.chanInfoList.list[num].idx = std::stoi(idx);
                                        tr.data.chanInfoList.list[num].rss = std::stoi(rss);
                                        tr.data.chanInfoList.list[num].snr = std::stoi(snr);
                                    }catch(...){
                                        break;
                                    }
                                    num++;
                                    
                                }
                            }
                        }
                    }

                    
                }
                

            }
            tr.data.chanInfoList.len =num;
            return tr;
        }
        case 5:
            tr.cmd = LoraCommandType::CHECK_STATUS;
            if(response.find(lora_chain_network_const::AT_RESPONSE_STATUS) != std::string::npos){
                if(response.find(lora_chain_network_const::AT_DEVICE_STATUS_IDLE) != std::string::npos){
                    tr.data.node_status = LoraNodeStatus::NODE_IDLE;
                }else if(response.find(lora_chain_network_const::AT_DEVICE_STATUS_TXRX) != std::string::npos){
                    tr.data.node_status = LoraNodeStatus::NODE_TXRX;
                }else if(response.find(lora_chain_network_const::AT_DEVICE_STATUS_SEARCHING) != std::string::npos){
                    tr.data.node_status = LoraNodeStatus::NODE_SEARCHING;
                }else if(response.find(lora_chain_network_const::AT_DEVICE_STATUS_BUZY) != std::string::npos){
                    tr.data.node_status = LoraNodeStatus::NODE_BUZY;
                }else{
                    tr.data.node_status = LoraNodeStatus::NODE_UNKNOWN;
                }
            }
            return tr;

        default:
            tr.cmd = LoraCommandType::UNKNOWN;
            return tr;

    }
}

ReceivedMesssageObj_t parseReceivedMessage(std::string msg){
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
 //                 ROS_WARN("Lora message parse failed");
                  return tr;
              }

              std::vector<uint8_t> data_vector = fromHexString(data,data.size()/2,0);
              if(data_vector.size() ==data.size()/2){
                tr.len = std::min(data_vector.size(),static_cast<size_t>(MAX_MESSAGE_PAYLOAD_LEN));
                memcpy(tr.payload,data_vector.data(),tr.len);
              }else{
 //               ROS_WARN("Lora message parse failed");
                return tr;
              }

              std::vector<uint8_t> qos_vector = fromHexString(qos,1,0);
              uint8_t qos_u8 =  qos_vector[0] & 0xF0;
              tr.type = static_cast<MessageType>(qos_u8);
              return tr;

            }
          }
        }
    }
  }
//  ROS_WARN("Lora message parse failed");
  return tr;
}