
#ifndef DRIVER_STIM300_DRIVER_STIM300_H
#define DRIVER_STIM300_DRIVER_STIM300_H


#include "../src/serial_unix.h"
#include "../src/lora_chain_network_constants.h"

enum class LoraReturnedStatus {
  RES_OK,
  RES_BUZY,
  RES_ERROR,
  RES_TIMEOUT,
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
}

struct ChanInfoList_t{
  ChanInfoObj_t list[MAX_CHAN_INFO_LEN];
  uint8_t len;
}

enum class MessageType{
  DATA_TRAFFIC,
  ACKNOWLEDGEMENT
}
static constexpr uint8_t MAX_MESSAGE_PAYLOAD_LEN = 200;

struct ReceivedMesssageObj_t {
  uint8_t Source[DEVICE_ID_LEN];
  uint8_t payload[MAX_MESSAGE_PAYLOAD_LEN];
  MessageType type;
  uint16_t len;
}

using ReceiveCallback = void(*)(ReceivedMesssageObj_t message);

class DriverStim300 {
public:
  DriverStim300(SerialDriver &serial_driver);
  ~DriverStim300() = default;
  // The class is Non-Copyable
  DriverStim300(const DriverStim300 &a) = delete;
  DriverStim300 &operator=(const DriverStim300 &a) = delete;
  // The class is non-movable
  DriverStim300(DriverStim300 &&a) = delete;
  DriverStim300 &operator=(DriverStim300 &&a) = delete;


  LoraReturnedStatus setID(uint8_t * id) const noexcept;
  LoraReturnedStatus softReset() const noexcept;
  LoraReturnedStatus join() const noexcept;
  LoraReturnedStatus status(LoraNodeStatus * status) const noexcept;
  LoraReturnedStatus send(uint8_t * target, QosLevel qos, uint8_t * payload, int len) const noexcept;
  LoraReturnedStatus getChanInfo(ChanInfoList_t * list) const noexcept;
  LoraReturnedStatus registerRXcallback(ReceiveCallback cb) const noexcept;



private:
  SerialDriver &serial_driver_;
  std::vector<uint8_t> rxbuffer_{};
  char txbuffer_[100+ MAX_MESSAGE_PAYLOAD_LEN*2];

  LoraReturnedStatus readOneLine();
  LoraReturnedStatus sendCommand(const std::vector<uint8_t>::const_iterator & begin);
  void emptyRxBuffer();
};

#endif // DRIVER_STIM300_DRIVER_STIM300_H
