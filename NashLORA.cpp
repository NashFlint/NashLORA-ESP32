#include "NashLORA.h"

// definitions - undecided if these should go here or in header

// register addresses - notes are for info or maybe for future development
#define REG_FIFO                    0x00
#define REG_OP_MODE                 0x01
#define REG_FR_MSB                  0x06
#define REG_FR_MID                  0x07
#define REG_FR_LSB                  0x08
#define REG_PA_CONFIG               0x09
#define REG_PA_RAMP                 0x0A // unsure if needed
#define REG_OCP                     0x0B // unsure if needed
#define REG_LNA                     0x0C
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_PTR        0x0E
#define REG_FIFO_RX_BASE_PTR        0x0F
#define REG_FIFO_RX_CURRENT_PTR     0x10
#define REG_IRQ_FLAGS_MASK          0x11 // unsure if needed
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NUM_BYTES            0x13
// skipped some registers for count of receveived things here, not sure if needed
#define REG_MODEM_STAT              0x18 // unsure if needed
#define REG_PACKET_SNR              0x19
#define REG_PACKET_RSSI             0x1A
// skipped register for current rssi - probably unneccesary with previous rssi register
// skipped hop channel register
#define REG_MODEM_CONFIG_1          0x1D
#define REG_MODEM_CONFIG_2          0x1E
// skipped register about symbol timeout
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LEN             0x22
// skipped some registers about max payload len and frequency hopping
#define REG_FIFO_RX_BYTE_ADDR_PTR   0x25 // unsure if needed
#define REG_MODEM_CONFIG_3          0x26
// skipped some registers about rf frequency error
#define REG_RSSI_WIDEBAND           0x2C // used to generate a random number - interesting
// skipped some whacky IFFreq unlabeled registers
#define REG_DETECTION_OPTIMIZE      0x31
// skipped register about inverting I and Q and optimizing for high bandwidth
#define REG_DETECTION_THRESHOLD     0x37
#define REG_SYNC_WORD               0x39
// skipped register about inverting I and Q and optimizing for high bandwidth part 2
#define REG_DIO_MAPPING_1           0x40 // used for doing things with DIO pins, not used in dogometer v1
#define REG_DIO_MAPPING_2           0x41
#define REG_VERSION                 0x42
// there are some other registers, seems they are not needed

// add more defines here for needed things possibly, like writing certain registers in repeated ways

// pins for LORA
#define PIN_MISO    GPIO_NUM_13
#define PIN_MOSI    GPIO_NUM_11
#define PIN_SCK     GPIO_NUM_12
//#define PIN_CS      GPIO_NUM_10
//#define PIN_RST     GPIO_NUM_8

// register settings for different modes - these are set to always keep the device in LoRa long range mode
#define MODE_SLEEP 0x80
#define MODE_STANDBY 0x81
#define MODE_TX 0x83
#define MODE_RX 0x86
#define MODE_RX_CONT 0x85 // should use RX Continuous mode unless precise arrival time for signal is known, basically just use this

// register settings for IRQ flags
#define IRQ_TX_DONE 0x08
#define IRQ_RX_DONE 0x40
#define IRQ_CRC_ERR 0x20

#define SPI_HOST_ID SPI2_HOST // this can be changed to support more SPI devices, probably unneeded

static const char *TAG = "LORA_DEBUG";

// bandwidths in kHz corresponding to int value needed to set them in register (0 -> 9)
static const double bandwidths[10] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};

// Initialize the ESP32 SPI bus, this must be called before initializing loras
void initSPI()
{
    spi_bus_config_t busConfig = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };

    spi_bus_initialize(SPI_HOST_ID, &busConfig, SPI_DMA_CH_AUTO);

    //vTaskDelay(500/portTICK_PERIOD_MS);
}

NashLORA::NashLORA(gpio_num_t cs, gpio_num_t rst)
{
    // set pin numbers
    csPin = cs;
    rstPin = rst;

    // set other values as default
    freq = 434000000;
    SF = 7;
    BW = 125;
}

bool NashLORA::init()
{
    // prep pins
    gpio_reset_pin(rstPin);
    gpio_reset_pin(csPin);

    gpio_set_direction(rstPin, GPIO_MODE_OUTPUT);
    gpio_set_level(rstPin, 1);
    //gpio_set_direction(csPin, GPIO_MODE_OUTPUT);
    //gpio_set_level(csPin, 1); // this line of code may be communist

    // config and initialize SPI

    // spi_device_interface_config_t devConfig = {0};

    // devConfig.mode = 0;
    // devConfig.clock_speed_hz = 9000000;
    // devConfig.spics_io_num = csPin;
    // devConfig.flags = 0;
    // devConfig.queue_size = 7;
    // devConfig.pre_cb = NULL;

    spi_device_interface_config_t devConfig = {
        .mode = 0,
        .clock_speed_hz = 8000000,
        .spics_io_num = csPin,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL
    };

    // probably should check for errors here
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_ID, &devConfig, &spiHandle));

    // reset chip
    reset();

    // verify the chip is communicating
    uint8_t version;
    version = readRegister(REG_VERSION);

    if (version != 0x12)
    {
        return false;
    }

    // put device into sleep mode - ensure in lora operating mode
    sleep();

    // set default parameters
    setFreq(915000000);
    setPower(17); // set to max power
    writeRegister(REG_LNA, 0x20 | 0x03); // boost LNA current to 150%

    // reset FIFO to 0
    writeRegister(REG_FIFO_RX_BASE_PTR, 0);
    writeRegister(REG_FIFO_TX_BASE_PTR, 0);

    enableAGC(true); // enable automatic gain control

    standby(); // go to standby mode

    return true;

}

void NashLORA::reset()
{
    gpio_set_level(rstPin, 0);
    uint64_t startTime = esp_timer_get_time();
    while (esp_timer_get_time() <= startTime + 100);
    //vTaskDelay(10/portTICK_PERIOD_MS); // according to docs this should be 100us but whatever this probably works
    gpio_set_level(rstPin, 1);
    startTime = esp_timer_get_time();
    while (esp_timer_get_time() <= startTime + 5*1000);
    //vTaskDelay(10/portTICK_PERIOD_MS); // according to docs this should only me 5ms but do 10ms to be safe
}

// add missing initializers here, Nash thinks something fucky wucky may be happening
uint8_t NashLORA::readRegister(uint8_t reg)
{
    uint8_t out[2] = {reg, 0xff};
    uint8_t in[2];

    spi_transaction_t trans = {
        .flags = 0,
        .length = 16,
        .tx_buffer = out,
        .rx_buffer = in
    };

    spi_device_transmit(spiHandle, &trans);

    return in[1];
}

void NashLORA::writeRegister(uint8_t reg, uint8_t val)
{
    uint8_t out[2];// = {reg | t, val}; // bitwise and with reg, puts a 10000000 to signify that its a write
    out[0] = 0x80 | reg;
    out[1] = val;
    uint8_t in[2];

    spi_transaction_t trans = {
        .flags = 0,
        .length = 16,
        .tx_buffer = out,
        .rx_buffer = in
    };

    spi_device_transmit(spiHandle, &trans);
}

void NashLORA::sleep()
{
    writeRegister(REG_OP_MODE, MODE_SLEEP);
}

void NashLORA::standby()
{
    writeRegister(REG_OP_MODE, MODE_STANDBY);
}

void NashLORA::setFreq(uint32_t frequency) // take in freq in Hz
{
    freq = frequency;
    uint32_t frf = freq * 0.016384; // simplified conversion from datasheet

    // write bytes by shifting and masking
    writeRegister(REG_FR_MSB, frf >> 16 & 0xff);
    writeRegister(REG_FR_MID, frf >> 8 & 0xff);
    writeRegister(REG_FR_LSB, frf >> 0 & 0xff);
}

void NashLORA::setPower(uint8_t power)
{
    // power is between 2 and 17 (dBm)
    if (power < 2)
    {
        power = 2;
    }
    if (power > 17)
    {
        power = 17;
    }

    writeRegister(REG_PA_CONFIG, 0x80 | 0x70 | (power - 2)); // set power register, unsure of 0x70, sets parameter called MaxPower to be its max which is still less than PA_BOOST and seems unused in other code
}

void NashLORA::send(uint8_t* buf, uint8_t len)
{
    standby(); // put radio into standby to transfer data
    writeRegister(REG_FIFO_ADDR_PTR, 0); // put pointer for FIFO to zero to be able to use max size

    // write buf to FIFO
    for (int i = 0; i < len; i++)
    {
        writeRegister(REG_FIFO, buf[i]);
    }
    writeRegister(REG_PAYLOAD_LEN, len);

    // put lora into send mode
    writeRegister(REG_OP_MODE, MODE_TX);
    // wait for lora to be done sending by reading flag for completion
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE) == 0)
    {
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    //reset IRQ flag by writing 1 to bit in register
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE);
    // reset register pointer -> not sure if needed
    writeRegister(REG_FIFO_ADDR_PTR, 0);
}

void NashLORA::receive()
{
    writeRegister(REG_OP_MODE, MODE_RX_CONT);
}

int NashLORA::received()
{
    if ((readRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE) == 0)
    {
        return 0;
    }
    else
    {
        if ((readRegister(REG_IRQ_FLAGS) & IRQ_CRC_ERR) == 0)
        {
            return 1;
        }
        else
        {
            return 2; // received packet with crc error    
        }
    }
}

void NashLORA::receivePacket(uint8_t* buf, uint8_t* len) // this function only receives the most recent packet
{
    // get len of received packet and reset rx flag
    standby();
    *len = readRegister(REG_RX_NUM_BYTES);
    writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE);
    writeRegister(REG_IRQ_FLAGS, IRQ_CRC_ERR);

    ESP_LOGI(TAG, "Current rx ptr = %02X", readRegister(REG_FIFO_RX_CURRENT_PTR));

    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_PTR)); // pointer to start of packet in FIFO

    for (int i = 0; i < *len; i++) // read FIFO into buffer for length of packet
    {
        buf[i] = readRegister(REG_FIFO); 
    }
    receive();
}

int NashLORA::getPacketRSSI()
{
    int rssi;
    if (freq >= 525000000)
    {
        rssi = -157 + readRegister(REG_PACKET_RSSI);
    }
    else
    {
        rssi = -164 + readRegister(REG_PACKET_RSSI);
    }

    return rssi;
}

bool NashLORA::signalDetected()
{
    if ((readRegister(REG_MODEM_STAT) & 0b00000001) == 0b00000001)
    {
        return true;
    }
    return false;
}

bool NashLORA::signalSynced()
{
    if ((readRegister(REG_MODEM_STAT) & 0b00000010) == 0b00000010)
    {
        return true;
    }
    return false;
}

bool NashLORA::rxOngoing()
{
    if ((readRegister(REG_MODEM_STAT) & 0b00000100) == 0b00000100)
    {
        return true;
    }
    return false;
}

bool NashLORA::headerValid()
{
    if ((readRegister(REG_MODEM_STAT) & 0b00001000) == 0b00001000)
    {
        return true;
    }
    return false;
}

uint32_t NashLORA::getFreq()
{
    return freq;
}

void NashLORA::enableCRC(bool en)
{
    uint8_t buf = readRegister(REG_MODEM_CONFIG_2);
    if (en)
    {
        writeRegister(REG_MODEM_CONFIG_2, buf | 0x04);
    }
    else
    {
        writeRegister(REG_MODEM_CONFIG_2, buf & 0xFB);
    }
}

void NashLORA::setGain(uint8_t Gval)
{
    // read register
    uint8_t buf = readRegister(REG_LNA);

    // clear bits for gain numbers
    for (int i = 5; i <= 7; i++)
    {
        buf = buf & ~(1 << i);
    }

    if (Gval < 1)
    {
        Gval = 1;
    }
    else if (Gval > 6)
    {
        Gval = 6;
    }

    buf = buf | (Gval << 5);

    writeRegister(REG_LNA, buf);
}

void NashLORA::enableAGC(bool en)
{
    uint8_t buf = readRegister(REG_MODEM_CONFIG_3);
    if (en)
    {
        buf = buf | 1 << 2;
    }
    else
    {
        buf = buf & ~(1 << 2);
    }

    writeRegister(REG_MODEM_CONFIG_3, buf);
}

void NashLORA::setCodingRate(uint8_t codingRate)
{
    // clamp to allowed values
    if (codingRate > 8)
    {
        codingRate = 8;
    }
    else if (codingRate < 5)
    {
        codingRate = 5;
    }

    // read register
    uint8_t buf = readRegister(REG_MODEM_CONFIG_1);

    // clear bits for coding rate in register
    for (int i = 1; i <= 3; i++)
    {
        buf = buf & ~(1 << i);
    }

    // make numbers match up
    uint8_t setVal = codingRate - 4;

    buf = buf | (setVal << 1);
    
    writeRegister(REG_MODEM_CONFIG_1, buf);

}

void NashLORA::setSpreadingFactor(uint8_t spreadingFactor)
{
    // clamp to allowed values
    if (spreadingFactor > 12)
    {
        spreadingFactor = 12;
    }
    else if (spreadingFactor < 6)
    {
        spreadingFactor = 6;
    }

    // read register
    uint8_t buf = readRegister(REG_MODEM_CONFIG_2);

    // clear bits for spreading factor in register
    for (int i = 4; i <= 7; i++)
    {
        buf = buf & ~(1 << i);
    }

    buf = buf | (spreadingFactor << 4);

    writeRegister(REG_MODEM_CONFIG_2, buf);

    SF = spreadingFactor;

    handleLowDataRateOptimization();
}

void NashLORA::setBandwidth(uint8_t bandwidthVal)
{
    // clamp bandwidth val to allowed values
    if (bandwidthVal > 9)
    {
        bandwidthVal = 9;
    }

    uint8_t buf = readRegister(REG_MODEM_CONFIG_1);

    //clear bits for bandwidth in register
    for (int i = 4; i <= 7; i++)
    {
        buf = buf & ~(1 << i);
    }

    buf = buf | (bandwidthVal << 4);

    writeRegister(REG_MODEM_CONFIG_1, buf);

    BW = bandwidths[bandwidthVal];

    handleLowDataRateOptimization();
}

void NashLORA::setSyncWord(uint8_t syncWord)
{
    writeRegister(REG_SYNC_WORD, syncWord);
}

double NashLORA::getSymbolPeriod()
{
    double symbolPeriod = 1/(BW/pow(2.0, SF));
    return symbolPeriod;
}

void NashLORA::handleLowDataRateOptimization()
{
    uint8_t buf = readRegister(REG_MODEM_CONFIG_3);

    if (getSymbolPeriod() >= 16.0)
    {
        writeRegister(REG_MODEM_CONFIG_3, buf | (1 << 3));
    }
    else
    {
        writeRegister(REG_MODEM_CONFIG_3, buf & ~(1 << 3));
    }
}

void NashLORA::clearReceiveFlags()
{
    writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE);
    writeRegister(REG_IRQ_FLAGS, IRQ_CRC_ERR);
}