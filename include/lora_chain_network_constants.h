#ifndef DRIVER_LORA_CHAIN_NETWORK_CONSTANTS_H
#define DRIVER_LORA_CHAIN_NETWORK_CONSTANTS_H

#include <array>
#include <cstdint>
#include <math.h>

namespace lora_chain_network_const {

static const std::string AT_CMD_RESET =     "AT+RESET";
static const std::string AT_CMD_JOIN  =     "AT+JOIN";
static const std::string AT_CMD_STATUS =    "AT+STATUS=";
static const std::string AT_CMD_SEND =      "AT+SEND";
static const std::string AT_CMD_CHANINFO =  "AT+CHAN_INFO";
static const std::string AT_CMD_SET_ID =    "AT+SET_ID";
static const std::string AT_CMD_REJOIN  =    "AT+REJOIN";
static const std::string AT_CMD_QUIT  =      "AT+QUIT";

static const std::string AT_DEVICE_STATUS_IDLE  =        "IDLE";
static const std::string AT_DEVICE_STATUS_SEARCHING  =   "SEARCHING";
static const std::string AT_DEVICE_STATUS_TXRX  =        "TXRX";
static const std::string AT_DEVICE_STATUS_BUZY  =        "BUZY";
static const std::string AT_DEVICE_STATUS_UNKNOWN  =     "UNKNOWN";

static const std::string AT_RETURN_RES_OK  =          "OK";
static const std::string AT_RETURN_RES_BUZY  =        "BUZY";
static const std::string AT_RETURN_RES_TIMEOUT  =     "TIMEOUT";
static const std::string AT_RETURN_RES_ERROR  =       "ERROR";
static const std::string AT_RETURN_RES_UNKNOWN  =     "UNKNOWN";
static const std::string AT_RETURN_RES_FORMAT_ERR  =  "COMMAND_FORMAT_ERROR";


static const std::string AT_RESPONSE_RES              =             "+RES=";
static const std::string AT_RESPONSE_STATUS           =             "+STATUS=";
static const std::string AT_RESPONSE_RECEIVE          =             "+RECEIVE=";
static const std::string AT_RESPONSE_EVENT          =               "+EVT=";

static const std::string AT_EVENT_DISCONNECT        =               "LOST_CONNECTION";
static const std::string AT_EVENT_CONNECTED        =                "CONNECTED";

} // namespace lora_chain_network_const
#endif // DRIVER_LORA_CHAIN_NETWORK_CONSTANTS_H
