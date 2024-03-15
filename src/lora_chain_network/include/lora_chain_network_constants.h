#ifndef DRIVER_LORA_CHAIN_NETWORK_CONSTANTS_H
#define DRIVER_LORA_CHAIN_NETWORK_CONSTANTS_H

#include <array>
#include <cstdint>
#include <math.h>

namespace lora_chain_network_const {

static constexpr std::string AT_CMD_RESET =     "AT+RESET";
static constexpr std::string AT_CMD_JOIN  =     "AT+JOIN";
static constexpr std::string AT_CMD_STATUS =    "AT+STATUS=";
static constexpr std::string AT_CMD_SEND =      "AT+SEND";
static constexpr std::string AT_CMD_CHANINFO =  "AT+CHAN_INFO";
static constexpr std::string AT_CMD_SET_ID =    "AT+SET_ID";


static constexpr std::string AT_DEVICE_STATUS_IDLE  =        "IDLE";
static constexpr std::string AT_DEVICE_STATUS_SEARCHING  =   "SEARCHING";
static constexpr std::string AT_DEVICE_STATUS_TXRX  =        "TXRX";
static constexpr std::string AT_DEVICE_STATUS_BUZY  =        "BUZY";
static constexpr std::string AT_DEVICE_STATUS_UNKNOWN  =     "UNKNOWN";

static constexpr std::string AT_RETURN_RES_OK  =          "OK";
static constexpr std::string AT_RETURN_RES_BUZY  =        "BUZY";
static constexpr std::string AT_RETURN_RES_TIMEOUT  =     "TIMEOUT";
static constexpr std::string AT_RETURN_RES_ERROR  =       "ERROR";
static constexpr std::string AT_RETURN_RES_UNKNOWN  =     "UNKNOWN";
static constexpr std::string AT_RETURN_RES_FORMAT_ERR  =  "COMMAND_FORMAT_ERROR";


static constexpr std::string AT_RESPONSE_RES              =             "+RES=";
static constexpr std::string AT_RESPONSE_STATUS           =             "+STATUS=";
static constexpr std::string AT_RESPONSE_RECEIVE          =             "+RECEIVE=";


} // namespace lora_chain_network_const
#endif // DRIVER_LORA_CHAIN_NETWORK_CONSTANTS_H
