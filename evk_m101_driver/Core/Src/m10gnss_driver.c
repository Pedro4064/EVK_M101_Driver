#include "i2c.h"
#include "m10gnss_driver.h"
#include "nmea_parser.h"


#define MESSAGE_START '$'
#define NUM_PARSING_TABLE_ENTRIES 2

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

void M10GnssDriverInit(m10_gnss* m10_module){
    m10_gnss_module = m10_module;
}

void M10GnssDriverNmeaDiscardMessage(void){

}

void M10GnssDriverNmeaMessageDelegator(nmea_caller_id* nmea_origin_id){
    for (int parsing_table_index = 0; parsing_table_index < NUM_PARSING_TABLE_ENTRIES; parsing_table_index++){
        nmea_message_parsing_table_entry nmea_callback_entry = nmea_message_parsing_table[parsing_table_index];

        char nmea_caller_compare_result = NmeaParserCompareOriginId(nmea_origin_id, nmea_callback_entry.message_origin);
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
        HAL_I2C_Mem_Read(m10_gnss_module->i2c_handle, m10_gnss_module->i2c_address, STREAM_BUFFER_REGISTER, STREAM_BUFFER_REGISTER_SIZE, &stream_buffer, 1, 1000);

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