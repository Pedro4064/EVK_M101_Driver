# EVK-M10 Driver

## Overview 

Driver for interfacing with [ublox's M10 GNSS Chip](https://www.u-blox.com/en/product/evk-m10) through $I^2C$ on the STM32 platform.

## Implemented Features
Currently the driver is capable of reading the M10's stream buffer through I2C Fast Mode and parsing the received data in the NMEA format, automatically handling message slicing and loosing stream buffer sync.

> [!IMPORTANT]  
> The driver is meat to be used with the NMEA message specification, and not UBX.

Currently the following NMEA messages have implemented parser functions:

1. `RMC (Recommended Minium Data)`: According to [ublox's interface description](https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf), section 2.7.17.1, the RMC message has the mos essential data for a GNSS system, such as, but not limited to: `Latitude`, `Longitude`, `Speed Over Ground`.

On how to implement new parser functions, please check this project's wiki, which goes deeper into implementation detail and driver architecture.

## How to Use

### Setup

To setup this system on the STM32 platform:

1. Put the `nmea_parser.c`, `nmea_parser.h`, `m10gnss_driver.c`, and `m10gnss_driver.h` files in your project's `Src` directory.
2. Enable the I2C peripheral.

### Example Use

This directory has an example in the `application.c` file under the `evk_m101_driver/Core/Src` file, but in short:

```c
#include "m10gnss_driver.h"
#include "application.h"
#include "gpio.h"
#include "i2c.h"

m10_gnss gnss_module = {
                            .i2c_address = I2C_ADDRESS,  // The address of the ublox module
                            .i2c_handle = &hi2c1         // Handler for configured I2C peripheral
                        };


void ApplicationMain(void){

    // Initializes the driver by clearing the ublox module's stream buffer
    M10GnssDriverInit(&gnss_module);
    while (1)
    {
        // Read the buffer and parse the RMC messages
        M10GnssDriverReadData();

        // If the latitude is available from the last reading, toggle the onboard LED
        if(gnss_module.latitude.is_available)
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

        HAL_Delay(100);
    }
    
}
```

## Porting to Another Platform

Since the whole parsing logic and conversion from NMEA string message to numerical values is all platform agnostic, it may be of interest to port this code to another platform other than an STM32 micro-controller. 

To do that, you only need to:
- Change the `void M10GnssDriverReadStreamBuffer(void)` function, which is the one responsible for making the I2C communication to the one specific to your application, in the `m10_gnss_driver.c` file.
- Change the `m10_gnss` struct, to either remove the handler to the I2C peripheral or use the one specific to you platform, in the `m10_gnss_driver.h` file.
- Remove the `#include "i2c.h"` directive from the `m10_gnss_driver.h` file.