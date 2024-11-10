#include "i2c.h"
#include "m10gnss_driver.h"

#define I2C_ADDRESS 0x84             // Default address for EVK-M101 module
#define STREAM_BUFFER_EMPTY 0xFF     // Value returned by stream buffer when empty
#define STREAM_BUFFER_REGISTER 0xFF  // Address of the stream buffer register
#define STREAM_BUFFER_REGISTER_SIZE 1  // Address of the stream buffer register

#define MESSAGE_START '$'
#define NUM_PARSING_TABLE_ENTRIES 2
#define NMEA_CALLER_ID_SIZE 5


typedef unsigned char nmea_caller_id[6];

typedef struct NMEA_MESSAGE_PARSING_TABLE_ENTRY
{
    nmea_caller_id message_origin;
    void(*parser_function)(nmea_caller_id*) ;
} nmea_message_parsing_table_entry;

void M10GnssDriverRmcParser(nmea_caller_id* nmea_origin_id);
void M10GnssDriverGsvParser(nmea_caller_id* nmea_origin_id);

nmea_message_parsing_table_entry nmea_message_parsing_table[NUM_PARSING_TABLE_ENTRIES] = {

                                                                {
                                                                    .message_origin = "GNRMC",
                                                                    .parser_function = M10GnssDriverRmcParser
                                                                },
                                                                {
                                                                    .message_origin = "**GSV",
                                                                    .parser_function = M10GnssDriverGsvParser
                                                                }
                                                            };

I2C_HandleTypeDef* i2c_handler;
m10_gnss* m10_gnss_module;

void M10GnssDriverInit(I2C_HandleTypeDef* i2c_handle, m10_gnss* m10_module){
    i2c_handler = i2c_handle;
    m10_gnss_module = m10_module;
}

void M10GnssDriverNmeaDiscardMessage(void){

}

char M10GnssDriverNmeaCompareOriginId(nmea_caller_id* message_origin, nmea_caller_id* table_origin){
    for (int i = 0; i < NMEA_CALLER_ID_SIZE; i++){

        if((*message_origin)[i] != (*table_origin)[i] && (*table_origin)[i] != '*')
            return 0;
    }
    
    return 1;
}

void M10GnssDriverNmeaMessageDelegator(nmea_caller_id* nmea_origin_id){
    for (int parsing_table_index = 0; parsing_table_index < NUM_PARSING_TABLE_ENTRIES; parsing_table_index++){
        nmea_message_parsing_table_entry nmea_callback_entry = nmea_message_parsing_table[parsing_table_index];

        char nmea_caller_compare_result = M10GnssDriverNmeaCompareOriginId(nmea_origin_id, nmea_callback_entry.message_origin);
        if(nmea_caller_compare_result != 0)
            continue;

        (*nmea_callback_entry.parser_function)(nmea_origin_id);
        return;
    }

    // If there was no match in the table, discard the incoming message
    M10GnssDriverNmeaDiscardMessage();
    
}

void M10GnssDriverReadData(void){
    unsigned char stream_buffer;
    nmea_caller_id message_origin;
    int nmea_caller_id_index;

    do{
        HAL_I2C_Mem_Read(i2c_handler, I2C_ADDRESS, STREAM_BUFFER_REGISTER, STREAM_BUFFER_REGISTER_SIZE, &stream_buffer, 1, 1000);

        if(stream_buffer ==  STREAM_BUFFER_EMPTY){
            m10_gnss_module->buffer_empty = 1;
            return;
        }
        else if(stream_buffer == '$'){
            nmea_caller_id_index = 0;
        }
        else if(stream_buffer == ','){
            M10GnssDriverNmeaMessageDelegator(message_origin);
        }
        else{
            message_origin[nmea_caller_id_index] = stream_buffer;
            nmea_caller_id_index++;
        }



    }
    while (stream_buffer != STREAM_BUFFER_EMPTY);
    
}

void M10GnssDriverRmcParser(nmea_caller_id* nmea_origin_id){

}

void M10GnssDriverGsvParser(nmea_caller_id* nmea_origin_id){
}