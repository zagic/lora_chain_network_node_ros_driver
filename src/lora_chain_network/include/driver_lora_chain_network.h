
#ifndef DRIVER_STIM300_DRIVER_STIM300_H
#define DRIVER_STIM300_DRIVER_STIM300_H



#include "lora_chain_network_constants.h"
#include "serial/serial.h"

static constexpr int SERIAL_BAUDRATE = 115200;





class DriverStim300 {
public:
  DriverStim300(serial::Serial &serial_handler);
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
  


private:
  serial::Serial ser_;
  std::vector<uint8_t> rxbuffer_{};
  char txbuffer_[100+ MAX_MESSAGE_PAYLOAD_LEN*2];

  LoraReturnedStatus readOneLine();
  LoraReturnedStatus sendCommand(char * data,size_t len);
  void emptyRxBuffer();
};

#endif // DRIVER_STIM300_DRIVER_STIM300_H
