#ifndef __M10_GNSS_DRIVER_H__
# define __M10_GNSS_DRIVER_H__

#include  "i2c.h"

#define I2C_ADDRESS 0x84             // Default address for EVK-M101 module
#define STREAM_BUFFER_EMPTY 0xFF     // Value returned by stream buffer when empty
#define STREAM_BUFFER_REGISTER 0xFF  // Address of the stream buffer register
#define STREAM_BUFFER_REGISTER_SIZE 1  // Address of the stream buffer register

typedef struct AVAILABLE_SATELITES_TABLE{
    unsigned char GP;
    unsigned char GL;
    unsigned char GA;
    unsigned char GB;
    unsigned char GI;
    unsigned char GQ;
} available_satelites_table;

typedef struct GNSS_NUMERIC_MEASUREMENT{
    char is_available;
    double value;
    char unit_of_measurement;
}  gnss_numeric_measurement;

typedef struct UTC_DATE_TIME{
    unsigned char year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    float second;

    char is_available;
} utc_date_time;

typedef struct GNSS_LAT_LONG_MEASUREMENT{
    char is_available;
    int degrees;
    float minutes;
    char indicator;
} gnss_lat_long_measurement;

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

void M10GnssDriverInit(m10_gnss* m10_module);

void M10GnssDriverReadData(void);

#endif