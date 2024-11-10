#include "nmea_parser.h"
#include "i2c.h"

char NmeaParserCompareOriginId(nmea_caller_id* message_origin, nmea_caller_id* table_origin){
    for (int i = 0; i < NMEA_CALLER_ID_SIZE; i++){

        if((*message_origin)[i] != (*table_origin)[i] && (*table_origin)[i] != '*')
            return 0;
    }
    
    return 1;
}

void NmeaGetNextFieldRaw(m10_gnss* m10_gnss_module, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]){
    char received_buffer;
    char finished_reading;

    for (int buffer_index = 0; buffer_index < NMEA_RAW_BUFFER_SIZE; buffer_index++)
    {
        // If finished reading the I2C buffer, make sure to clear the user provided buffer to avoid contamination from previous runs
        if(finished_reading){
            (*raw_stream_buffer)[buffer_index] = '\0';
            continue;
        }

        HAL_I2C_Mem_Read(m10_gnss_module->i2c_handle, m10_gnss_module->i2c_address, STREAM_BUFFER_REGISTER, STREAM_BUFFER_REGISTER_SIZE, &received_buffer, 1, 1000);
        if(received_buffer == ',' || received_buffer == '\r' || received_buffer == '\n'){
            finished_reading = 1;
            continue;
        }
        else{
            (*raw_stream_buffer)[buffer_index] = received_buffer;
        }

    }
    
}