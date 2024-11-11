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
        if(received_buffer == ','){
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

    date_time->second  = CHAR_TO_NUMERIC(raw_stream_buffer, 4) * 10;
    date_time->second += CHAR_TO_NUMERIC(raw_stream_buffer, 5);
    date_time->second += CHAR_TO_NUMERIC(raw_stream_buffer, 7) / 10;
    date_time->second += CHAR_TO_NUMERIC(raw_stream_buffer, 8) / 100;
}