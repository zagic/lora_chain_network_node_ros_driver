
#include "driver_lora_chain_network.h"
#include <iostream>
#include <chrono>

DriverStim300::DriverStim300(serial::Serial &serial_handler)
    : ser_(serial_handler) {}




LoraReturnedStatus DriverStim300::readOneLine(){
  // auto start = std::chrono::high_resolution_clock::now();
  // while (true) {
  //   auto now = std::chrono::high_resolution_clock::now();
  //   std::chrono::duration<double> elapsed = now - start;
  //   if (elapsed.count() >= 2.0) {
  //       break;
  //   }
  // }
  std::string line;
  line = my_serial.readline(100+ MAX_MESSAGE_PAYLOAD_LEN*2, "\n");

}

LoraReturnedStatus DriverStim300::sendCommand(char * data,size_t len){
    ser.write(data,size_t len);
}

void DriverStim300::emptyRxBuffer(){

}


