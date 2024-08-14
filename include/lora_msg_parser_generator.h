#ifndef LORA_MSG_PARSER_GENERATOR_H
#define LORA_MSG_PARSER_GENERATOR_H

#include "lora_chain_network_constants.h"
#include <vector>


enum class LoraCommandType {
    SET_ID,
    JOIN,
    RESET,
    SEND,
    CHAN_INFO,
    CHECK_STATUS,
    REJOIN,
    QUIT,
    UNKNOWN

};

enum class LoraReturnedStatus {
  RES_OK,
  RES_BUZY,
  RES_ERROR,
  RES_TIMEOUT,
  RES_UNKNOWN,
  RES_COMMAND_FORMAT_ERROR,
  RES_UART_NO_RESPONSE,
};

enum class LoraNodeStatus {
  NODE_IDLE,
  NODE_BUZY,
  NODE_SEARCHING,
  NODE_TXRX,
  NODE_UNKNOWN
};

enum class QosLevel {
  QOS_LOW = 0,
  QOS_LOW_ACK = 2,
  QOS_MED = 4,
  QOS_MED_2ACK = 6,
  QOS_HIGH = 8
  // QOS_ULTRA_HIGH = 12, **reserverd
};
static constexpr uint8_t DEVICE_ID_LEN = 2;

static constexpr uint8_t MAX_CHAN_INFO_LEN = 8;

struct ChanInfoObj_t {
  uint8_t id[DEVICE_ID_LEN];
  uint8_t idx;
  int8_t snr;
  int16_t rss;
};

struct ChanInfoList_t{
  ChanInfoObj_t list[MAX_CHAN_INFO_LEN];
  uint8_t len;
};

enum class MessageType{
  DATA_TRAFFIC    =1<<4,
  ACKNOWLEDGEMENT =5<<4
};
static constexpr int MAX_MESSAGE_PAYLOAD_LEN = 200;

struct ReceivedMesssageObj_t {
  uint8_t Source[DEVICE_ID_LEN];
  uint8_t payload[MAX_MESSAGE_PAYLOAD_LEN];
  MessageType type;
  uint16_t len;
};

struct CommandResult_t{
    LoraCommandType cmd;
    LoraReturnedStatus status;
    union{
        uint16_t session_id;
        ChanInfoList_t chanInfoList;
        LoraNodeStatus node_status;
    }data;
};

std::string formSetIDCommand(uint8_t * id) ;
std::string formSoftResetCommand() ;
std::string formJoinCommand() ;
std::string formStatusCheckCommand() ;
std::string formSendCommand(uint8_t * target, QosLevel qos, uint8_t * payload, int len) ;
std::string formChanInfoCommand() ;
std::string formRejoinCommand() ;
std::string formQuitCommand() ;

CommandResult_t parseCommandResult(int command, int res, std::string response);
ReceivedMesssageObj_t parseReceivedMessage(std::string msg);

std::vector<uint8_t> fromHexString(const std::string& hexStr, size_t bytelen, size_t startPos = 0 );
std::string toHexString(const uint8_t* data, size_t length);

#endif