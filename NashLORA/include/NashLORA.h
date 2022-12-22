#ifndef NashLORA_H
#define NashLORA_H

// this will only work for ESP32(S3) - will use functions from ESP-IDF

#include <stdint.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

class NashLORA
{
    public:

    // can add error handling to these to make them only work if inited, but im not a pussy
    bool init(); // init chip 
    void reset(); // reset lora chip by pulling rst pin low then high
    void sleep(); // low power mode, lose FIFO
    void standby(); // set chip to idle in order to access FIFO and alter registers
    void send(uint8_t* buf, uint8_t len);
    void receive(); // set chip into receive mode
    int received(); // check to see if a packet has been received, 0 is no, 1 is yes, 2 is yes with crc error
    void receivePacket(uint8_t* buf, uint8_t* len); // receive packet
    void setPower(uint8_t power); // set transmit power in dBm

    void setFreq(uint32_t freq);

    uint8_t readRegister(uint8_t reg);
    
    private:
    
    void writeRegister(uint8_t reg, uint8_t val); // write value to register
     // read register value
    
    spi_device_handle_t spiHandle;
};

#endif