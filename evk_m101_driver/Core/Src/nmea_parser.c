#include <stdlib.h>

#include "nmea_parser.h"
#include "m10gnss_driver.h"
#include "i2c.h"

#define CHAR_TO_NUMERIC(char_buffer, position) (int)(((*char_buffer)[position])-48)

char NmeaParserCompareOriginId(nmea_caller_id* message_origin, nmea_caller_id* table_origin){
    for (int i = 0; i < NMEA_CALLER_ID_SIZE; i++){

        if((*message_origin)[i] != (*table_origin)[i] && (*table_origin)[i] != '*')
            return 0;
    }
    
    return 1;
}

nmea_raw_field_metadata NmeaGetNextFieldRaw(m10_gnss* m10_gnss_module, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]){
    char received_buffer;
    char finished_reading;
    char end_of_message;
    nmea_raw_field_metadata metadata;

    for (int buffer_index = 0; buffer_index < NMEA_RAW_BUFFER_SIZE; buffer_index++)
    {
        // If finished reading the I2C buffer, make sure to clear the user provided buffer to avoid contamination from previous runs
        if(finished_reading){
            (*raw_stream_buffer)[buffer_index] = '\0';
            continue;
        }

        HAL_I2C_Mem_Read(m10_gnss_module->i2c_handle, m10_gnss_module->i2c_address, STREAM_BUFFER_REGISTER, STREAM_BUFFER_REGISTER_SIZE, &received_buffer, 1, 1000);
        if(received_buffer == ',' || received_buffer == 0xFF){
            finished_reading = 1;
        }
        else if(received_buffer == '\r'){
            // continue to get the next end of message character
            continue;
        }
        else if(received_buffer == '\n'){
            finished_reading = 1;
            end_of_message = 1;
        }
        else{
            (*raw_stream_buffer)[buffer_index] = received_buffer;
            metadata.raw_field_length = buffer_index;
        }
    }

    metadata.field_status = (end_of_message)?END_OF_MESSAGE:metadata.raw_field_length == 0;
    return metadata;
}

void NmeaParseUtcTime(utc_date_time* date_time, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]){
    date_time->hour  = CHAR_TO_NUMERIC(raw_stream_buffer, 0) * 10;
    date_time->hour += CHAR_TO_NUMERIC(raw_stream_buffer, 1);

    date_time->minute  = CHAR_TO_NUMERIC(raw_stream_buffer, 2) * 10;
    date_time->minute += CHAR_TO_NUMERIC(raw_stream_buffer, 3);

    date_time->second  = CHAR_TO_NUMERIC(raw_stream_buffer, 4) * 10.0;
    date_time->second += CHAR_TO_NUMERIC(raw_stream_buffer, 5) * 1.0;
    date_time->second += CHAR_TO_NUMERIC(raw_stream_buffer, 7) / 10.0;
    date_time->second += CHAR_TO_NUMERIC(raw_stream_buffer, 8) / 100.0;
}

void NmeaParseUtcDate(utc_date_time* date_time, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]){
    date_time->day  = CHAR_TO_NUMERIC(raw_stream_buffer, 0) * 10;
    date_time->day += CHAR_TO_NUMERIC(raw_stream_buffer, 1);

    date_time->month  = CHAR_TO_NUMERIC(raw_stream_buffer, 2) * 10;
    date_time->month += CHAR_TO_NUMERIC(raw_stream_buffer, 3);


    date_time->year  = CHAR_TO_NUMERIC(raw_stream_buffer, 4) * 10;
    date_time->year += CHAR_TO_NUMERIC(raw_stream_buffer, 5);
}

void NmeaParseLatLong(gnss_lat_long_measurement* lat_long_measurement, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE], nmea_lat_long_parser nmea_parser_option){
    int buffer_position = (nmea_parser_option==LATITUDE)? 0:1;

    lat_long_measurement->degrees = CHAR_TO_NUMERIC(raw_stream_buffer, buffer_position) * 10;
    lat_long_measurement->degrees+= CHAR_TO_NUMERIC(raw_stream_buffer, ++buffer_position) * 1;
    lat_long_measurement->degrees = (nmea_parser_option==LONGITUDE)?CHAR_TO_NUMERIC(raw_stream_buffer, 0) * 100 : lat_long_measurement->degrees;

    lat_long_measurement->minutes = CHAR_TO_NUMERIC(raw_stream_buffer, ++buffer_position) * 10.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_stream_buffer, ++buffer_position) * 1.0;
    buffer_position++; // Skip the . decimal separator
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_stream_buffer, ++buffer_position) / 10.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_stream_buffer, ++buffer_position) / 100.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_stream_buffer, ++buffer_position) / 1000.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_stream_buffer, ++buffer_position) / 10000.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_stream_buffer, ++buffer_position) / 100000.0;
}

double NmeaParseNumericFloatingPoint(char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]){
    return atof((const char*)raw_stream_buffer);
}