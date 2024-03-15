#include "lora_msg_parser_generator.h"

#include <sstream>
#include <iomanip>

std::string toHexString(const uint8_t* data, size_t length) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for(size_t i = 0; i < length; ++i) {
        ss << std::setw(2) << static_cast<int>(data[i]);
    }
    return ss.str();
}

std::vector<uint8_t> fromHexString(const std::string& hexStr, size_t bytelen, size_t startPos = 0 ) {
    std::vector<uint8_t> data;

    if (startPos >= hexStr.length() || startPos % 2 != 0) {
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
std::string formSetIDCommand(uint8_t * id) const noexcept{
  return AT_CMD_SET_ID + toHexString(id, DEVICE_ID_LEN) +",0000" +"/r/n";
}


/**
 * @brief Soft reset command
 * @return Generated command string
 */
std::string formSoftResetCommand() const noexcept{
    return AT_CMD_RESET+"/r/n";
}


/**
 * @brief Join netwokr command
 * @return Generated command string
 */
std::string formJoinCommand() const noexcept{
    return AT_CMD_JOIN+"/r/n";
}

/**
 * @brief status check command
 * @return Generated command string
 */
std::string formStatusCheckCommand()(LoraNodeStatus * status) const noexcept{
    return AT_CMD_STATUS+"?/r/n";
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
std::string formSendCommand(uint8_t * target, QosLevel qos, uint8_t * payload, int len) const noexcept{
    return AT_CMD_SEND + toHexString(target, DEVICE_ID_LEN) + ","+ toHexString(static_cast<uint8_t>(qos), 1)
    +",0,"+toHexString(payload, len)+"/r/n";
}

/**
 * @brief request channel info command
 * @return Generated command string
 */
std::string formChanInfoCommand() const noexcept{
    return AT_CMD_CHANINFO+"=0/r/n";
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
            tr.status = LoraReturnedStatus::RES_COMMAND_FORMAT_ERROR;
            break;
        case 5:
            tr.status = LoraReturnedStatus::RES_UART_NO_RESPONSE;
            break;
        default:
            tr.status = LoraReturnedStatus::RES_ERROR;
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
            if(response.find(AT_RESPONSE_STATUS) != std::string::npos){
                std::string token,session_id;
                if(std::getline(iss,token, ',')){
                    if(std::getline(iss,session_id, ',')){
                        std::vector<uint8_t> sesion_vector = fromHexString(session_id,2,0);
                        if(sesion_vector.size==2){
                            tr.data.session_id = sesion_vector[0]+sesion_vector[1]*256;
                        }else{
                            tr.status = LoraReturnedStatus::RES_ERROR;
                        }
                        
                    }
                }
            }
            return tr;
        case 4:
            tr.cmd = LoraCommandType::CHAN_INFO;
            uint8_t num =0;
            if(response.find(AT_RESPONSE_RECEIVE) != std::string::npos){
                std::istringstream iss(response);
                std::string token,id,idx,rss,snr;
                
                if(std::getline(iss, token, ' ')){   /*There is a space after +res= */
                    while(true){
                        if(std::getline(iss,id, ',')){
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
                                        try(
                                            tr.data.chanInfoList.list[num].idx = std::stoi(idx);
                                            tr.data.chanInfoList.list[num].rss = std::stoi(rss);
                                            tr.data.chanInfoList.list[num].snr = std::stoi(snr);
                                        )catch(...){
                                            break;
                                        }
                                        num++;
                                        
                                    }
                                }
                            }
                        }

                    }
                }
                

            }
            tr.data.chanInfoList.len =num;
            return tr;
        case 5:
            tr.cmd = LoraCommandType::CHECK_STATUS;
            if(response.find(AT_RESPONSE_STATUS) != std::string::npos){
                if(response.find(AT_DEVICE_STATUS_IDLE) != std::string::npos){
                    tr.data.node_status = LoraNodeStatus::IDLE;
                }else if(response.find(AT_DEVICE_STATUS_TXRX) != std::string::npos){
                    tr.data.node_status = LoraNodeStatus::TXRX;
                }else if(response.find(AT_DEVICE_STATUS_SEARCHING) != std::string::npos){
                    tr.data.node_status = LoraNodeStatus::SEARCHING;
                }else if(response.find(AT_DEVICE_STATUS_BUZY) != std::string::npos){
                    tr.data.node_status = LoraNodeStatus::BUZY;
                }else{
                    tr.data.node_status = LoraNodeStatus::UNKNOWN;
                }
            }
            return tr;

        default:
            tr.cmd = LoraCommandType::UNKNOWN;
            return tr;

    }
}