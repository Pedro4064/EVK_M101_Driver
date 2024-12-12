#ifndef __M10_GNSS_DRIVER_H__
# define __M10_GNSS_DRIVER_H__

#include  "i2c.h"

#define I2C_ADDRESS 0x84             // Default address for EVK-M101 module
#define STREAM_BUFFER_EMPTY 0xFF     // Value returned by stream buffer when empty
#define STREAM_BUFFER_REGISTER 0xFF  // Address of the stream buffer register
#define STREAM_BUFFER_REGISTER_SIZE 1  // Address of the stream buffer register

#define STACK_BUFFER_ARRAY_SIZE 400

/**
 * @brief Struct to store the number of available satelites for each possible constellation, used
 * which is possible to get by parsing the `GSV` message.
 * 
 */
typedef struct AVAILABLE_SATELITES_TABLE{
    unsigned char GP;
    unsigned char GL;
    unsigned char GA;
    unsigned char GB;
    unsigned char GI;
    unsigned char GQ;
} available_satelites_table;

/**
 * @brief Base struct for all numerical measurement from the GNSS module, containing 
 * relevant metadata.
 * 
 */
typedef struct GNSS_NUMERIC_MEASUREMENT{
    char is_available;         // Check if measurement was available in the last reading
    double value;              // Last available values
    char unit_of_measurement;  // Engineering Unit of measurement
}  gnss_numeric_measurement;

/**
 * @brief Struct containing UTC data time information, and relevant metadata.
 * 
 */
typedef struct UTC_DATE_TIME{
    unsigned char year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    float second;

    char is_available;
} utc_date_time;

/**
 * @brief Struct to hold data from both latitude and longitude.
 * 
 */
typedef struct GNSS_LAT_LONG_MEASUREMENT{
    char is_available;
    int degrees;
    float minutes;
    char indicator;
} gnss_lat_long_measurement;

/**
 * @brief Struct with all the necessary data for the working of the GNSS module as well as its readings.
 * 
 */
typedef struct M10_GNSS{
    available_satelites_table num_available_satelites;
    gnss_lat_long_measurement latitude;
    gnss_lat_long_measurement longitude;
    gnss_numeric_measurement course_over_ground;
    gnss_numeric_measurement speed_over_ground_knots;
    utc_date_time time_of_sample;
    char buffer_empty;
    
    I2C_HandleTypeDef* i2c_handle;
    int i2c_address;
} m10_gnss;

/**
 * @brief Struct holding the received stream buffer, as well as relevant metadata for data parsing.
 * 
 */
typedef struct M10_GNSS_STREAM_BUFFER{
    unsigned char buffer[STACK_BUFFER_ARRAY_SIZE];
    uint16_t buffer_size;
    int buffer_index;
} m10_gnss_stream_buffer;

/**
 * @brief Initialize the M10 GNSS Driver.
 * 
 * @param m10_module: `m10_gnss*` Pointer to an instance of m10_gnss
 */
void M10GnssDriverInit(m10_gnss* m10_module);

/**
 * @brief Read and parse the data on the module's stream buffer.
 * 
 */
void M10GnssDriverReadData(void);

/**
 * @brief Clear the module's stream buffer.
 * 
 */
void M10GnssDriverClearStreamBuffer(void);
#endif