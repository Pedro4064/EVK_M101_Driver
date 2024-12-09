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

nmea_raw_field_metadata NmeaGetNextFieldRaw(m10_gnss_stream_buffer* stream_buffer, char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]){
    char received_character = '0';
    char finished_reading = 0;
    char end_of_message = 0;
    nmea_raw_field_metadata metadata = {
                                        .raw_field_length = 0,
                                        .field_status = VALID
                                    };

    static int buffer_index = 0;
    while(buffer_index < NMEA_RAW_BUFFER_SIZE){

        if(finished_reading){
            // If finished reading the I2C buffer, make sure to clear the user provided buffer to avoid contamination from previous runs
            (*raw_field_buffer)[buffer_index] = '\0';
            buffer_index++;
            continue;
        }
        
        if(stream_buffer->buffer_index >= stream_buffer->buffer_size){
            metadata.field_status = PARSING_EN_ROUTE;
            return metadata;
        }

        received_character = stream_buffer->buffer[stream_buffer->buffer_index];
        stream_buffer->buffer_index++;

        if(received_character == ',' || received_character == 0xFF){
            finished_reading = 1;
        }
        else if(received_character == '\r'){
            // continue to get the next end of message character
            continue;
        }
        else if(received_character == '\n'){
            finished_reading = 1;
            end_of_message = 1;
        }
        else{
            (*raw_field_buffer)[buffer_index] = received_character;
            metadata.raw_field_length = buffer_index + 1;
            buffer_index++;
        }
    
    }

    metadata.field_status = (end_of_message)?END_OF_MESSAGE:metadata.raw_field_length == 0;
    buffer_index = 0;
    return metadata;
}

void NmeaParseUtcTime(utc_date_time* date_time, char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]){
    date_time->hour  = CHAR_TO_NUMERIC(raw_field_buffer, 0) * 10;
    date_time->hour += CHAR_TO_NUMERIC(raw_field_buffer, 1);

    date_time->minute  = CHAR_TO_NUMERIC(raw_field_buffer, 2) * 10;
    date_time->minute += CHAR_TO_NUMERIC(raw_field_buffer, 3);

    date_time->second  = CHAR_TO_NUMERIC(raw_field_buffer, 4) * 10.0;
    date_time->second += CHAR_TO_NUMERIC(raw_field_buffer, 5) * 1.0;
    date_time->second += CHAR_TO_NUMERIC(raw_field_buffer, 7) / 10.0;
    date_time->second += CHAR_TO_NUMERIC(raw_field_buffer, 8) / 100.0;
}

void NmeaParseUtcDate(utc_date_time* date_time, char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]){
    date_time->day  = CHAR_TO_NUMERIC(raw_field_buffer, 0) * 10;
    date_time->day += CHAR_TO_NUMERIC(raw_field_buffer, 1);

    date_time->month  = CHAR_TO_NUMERIC(raw_field_buffer, 2) * 10;
    date_time->month += CHAR_TO_NUMERIC(raw_field_buffer, 3);


    date_time->year  = CHAR_TO_NUMERIC(raw_field_buffer, 4) * 10;
    date_time->year += CHAR_TO_NUMERIC(raw_field_buffer, 5);
}

void NmeaParseLatLong(gnss_lat_long_measurement* lat_long_measurement, char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE], nmea_lat_long_parser nmea_parser_option){
    int buffer_position = (nmea_parser_option==LATITUDE)? 0:1;

    lat_long_measurement->degrees = CHAR_TO_NUMERIC(raw_field_buffer, buffer_position) * 10;
    lat_long_measurement->degrees+= CHAR_TO_NUMERIC(raw_field_buffer, ++buffer_position) * 1;
    lat_long_measurement->degrees = (nmea_parser_option==LONGITUDE)? lat_long_measurement->degrees + CHAR_TO_NUMERIC(raw_field_buffer, 0) * 100 : lat_long_measurement->degrees;

    lat_long_measurement->minutes = CHAR_TO_NUMERIC(raw_field_buffer, ++buffer_position) * 10.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_field_buffer, ++buffer_position) * 1.0;
    buffer_position++; // Skip the . decimal separator
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_field_buffer, ++buffer_position) / 10.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_field_buffer, ++buffer_position) / 100.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_field_buffer, ++buffer_position) / 1000.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_field_buffer, ++buffer_position) / 10000.0;
    lat_long_measurement->minutes+= CHAR_TO_NUMERIC(raw_field_buffer, ++buffer_position) / 100000.0;
}

double NmeaParseNumericFloatingPoint(char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]){
    return atof((const char*)raw_field_buffer);
}
