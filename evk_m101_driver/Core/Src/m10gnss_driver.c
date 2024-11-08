#include "i2c.h"
#include "m10gnss_driver.h"

#define I2C_ADDRESS 0x84             // Default address for EVK-M101 module
#define STREAM_BUFFER_REGISTER 0xFF  // Address of the stream buffer register


I2C_HandleTypeDef* i2c_handler;
m10_gnss* m10_gnss_module;

void M10GnssDriverInit(I2C_HandleTypeDef* i2c_handle, m10_gnss* m10_module){
    i2c_handler = i2c_handle;
    m10_gnss_module = m10_module;
}