#ifndef NashLORA_H
#define NashLORA_H

// this will only work for ESP32(S3) - will use functions from ESP-IDF

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"

void initSPI();

class NashLORA
{
    public:
    NashLORA(gpio_num_t cs, gpio_num_t rst);

    bool init(); // init chip 
    void reset(); // reset lora chip by pulling rst pin low then high
    void sleep(); // low power mode, lose FIFO
    void standby(); // set chip to idle in order to access FIFO and alter registers
    void send(uint8_t* buf, uint8_t len);
    void receive(); // set chip into receive mode
    int received(); // check to see if a packet has been received, 0 is no, 1 is yes, 2 is yes with crc error
    void receivePacket(uint8_t* buf, uint8_t* len); // receive packet
    void clearReceiveFlags(); // clear irq flags for packet received, can be used to ignore packet with crc error
    int getPacketRSSI();
    void setPower(uint8_t power); // set transmit power in dBm
    void setGain(uint8_t Gval); // set gain, 1 is max, 6 is min
    void enableAGC(bool en); // enable or disable auto gain control
    void setSyncWord(uint8_t syncWord); // set sync word - should not use 0x34

    void setCodingRate(uint8_t codingRate); // set coding rate, valid values are 5 -> 8
    void setSpreadingFactor(uint8_t spreadingFactor); // set spreading factor, valid values are 7 -> 12
    // I may need to do optimize for low data rate if spreading factor is too low

    void setBandwidth(uint8_t bandwidthVal); // set bandwidth, valid values are 0 -> 9

    void setFreq(uint32_t frequency);
    uint32_t getFreq();

    double getSymbolPeriod(); // returns time length of symbol in ms
    void handleLowDataRateOptimization(); // enables or disables low data rate optimization depending on symbol period

    bool signalDetected(); // return true if status says signal detected
    bool signalSynced(); // return true if status says signal is synchronized
    bool rxOngoing(); // return true if rx ongoing status is true
    bool headerValid(); // return true if status shows header valid received
    void enableCRC(bool en); // enable or disable crc error checking

    uint8_t readRegister(uint8_t reg); // read register value
    
    private:
    
    void writeRegister(uint8_t reg, uint8_t val); // write value to register

    uint32_t freq;
    uint8_t SF; // spreading factor
    double BW; // bandwidth
    
    gpio_num_t rstPin;
    gpio_num_t csPin;
    spi_device_handle_t spiHandle;// = {0};
};

#endif