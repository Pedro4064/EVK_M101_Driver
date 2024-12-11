#include "i2c.h"
#include "m10gnss_driver.h"
#include "nmea_parser.h"

#define AVAILABLE_BUFFER_HB 0xFD
#define AVAILABLE_BUFFER_LB 0xFE

#define MESSAGE_START '$'
#define NUM_PARSING_TABLE_ENTRIES 2

typedef struct NMEA_MESSAGE_PARSING_TABLE_ENTRY
{
    nmea_caller_id message_origin;
    void(*parser_function)(nmea_caller_id*) ;
} nmea_message_parsing_table_entry;

typedef enum PARSER_STATE{
    IDLE,
    PARSING
} parser_state;

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

m10_gnss* m10_gnss_module;
m10_gnss_stream_buffer raw_stream_buffer;

parser_state raw_stream_buffer_parser_state = IDLE;
nmea_caller_id message_origin;

void M10GnssDriverInit(m10_gnss* m10_module){
    m10_gnss_module = m10_module;
}

void M10GnssDriverNmeaDiscardMessage(void){

}

void M10GnssDriverNmeaMessageDelegator(nmea_caller_id* nmea_origin_id){
    for (int parsing_table_index = 0; parsing_table_index < NUM_PARSING_TABLE_ENTRIES; parsing_table_index++){
        nmea_message_parsing_table_entry nmea_callback_entry = nmea_message_parsing_table[parsing_table_index];

        char nmea_caller_compare_result = NmeaParserCompareOriginId(nmea_origin_id, nmea_callback_entry.message_origin);
        if(nmea_caller_compare_result == 0)
            continue;

        (*nmea_callback_entry.parser_function)(nmea_origin_id);
        return;
    }

    // If there was no match in the table, discard the incoming message
    M10GnssDriverNmeaDiscardMessage();
    
}

uint16_t M10GnssDriverGetStreamBufferSize(void){
        
        unsigned char raw_buffer_val;
        uint16_t buffer_size = 0;

        HAL_I2C_Mem_Read(m10_gnss_module->i2c_handle, m10_gnss_module->i2c_address, AVAILABLE_BUFFER_HB, STREAM_BUFFER_REGISTER_SIZE, &raw_buffer_val, 1, 10000);
        buffer_size |= raw_buffer_val<<4;

        HAL_I2C_Mem_Read(m10_gnss_module->i2c_handle, m10_gnss_module->i2c_address, AVAILABLE_BUFFER_LB, STREAM_BUFFER_REGISTER_SIZE, &raw_buffer_val, 1, 10000);
        buffer_size |= raw_buffer_val;
        return buffer_size;
}

void M10GnssDriverReadStreamBuffer(void){

        raw_stream_buffer.buffer_size = M10GnssDriverGetStreamBufferSize();
        raw_stream_buffer.buffer_size = (raw_stream_buffer.buffer_size > STACK_BUFFER_ARRAY_SIZE)?STACK_BUFFER_ARRAY_SIZE:raw_stream_buffer.buffer_size;
        HAL_I2C_Mem_Read(m10_gnss_module->i2c_handle, m10_gnss_module->i2c_address, STREAM_BUFFER_REGISTER, STREAM_BUFFER_REGISTER_SIZE, &raw_stream_buffer.buffer, raw_stream_buffer.buffer_size, 10000);
        raw_stream_buffer.buffer_index = 0;
}

void M10GnssDriverParseBuffer(void){
    static int nmea_caller_id_index;

    while(raw_stream_buffer.buffer_index < raw_stream_buffer.buffer_size){

        unsigned char stream_character = raw_stream_buffer.buffer[raw_stream_buffer.buffer_index];
        raw_stream_buffer.buffer_index++;

        if(stream_character == '$'){
            nmea_caller_id_index = 0;
        }
        else if(stream_character == ','){
            M10GnssDriverNmeaMessageDelegator(message_origin);
        }
        else{
            message_origin[nmea_caller_id_index] = stream_character;
            nmea_caller_id_index++;
        }
    }
    
}

void M10GnssDriverReadData(void){

    M10GnssDriverReadStreamBuffer();

    if(raw_stream_buffer.buffer_size == 0)
        return;

    // If the first element is $, force the state back to idle, to avoid parsing error propagation
    if(raw_stream_buffer.buffer[0] == '$')
        raw_stream_buffer_parser_state = IDLE;

    switch (raw_stream_buffer_parser_state){
        case IDLE:
            M10GnssDriverParseBuffer();
            break;

        case PARSING:
            M10GnssDriverNmeaMessageDelegator(message_origin);
            break;
        
        default:
            break;
    }
}


void M10GnssDriverRmcParser(nmea_caller_id* nmea_origin_id){

    static char raw_field_data[20];           // Buffer containing the raw NMEA field
    static int field_index;                   // Index of the field being parsed at the moment
    
    raw_stream_buffer_parser_state = PARSING; // Set the parser state to PARSING, so if message is cut due to buffer limit, resume parsing here

    while(field_index < 15){

        nmea_raw_field_metadata field_metadata = NmeaGetNextFieldRaw(&raw_stream_buffer, &raw_field_data);
        switch (field_index){

            case 0:
                if(field_metadata.field_status == PARSING_EN_ROUTE){
                    // If parsing en route but the message was cut due to buffer size constraints, just return to ParseBuffer function with the
                    // parser state still as PARSING, and field index as 0, so it will continue the parsing here
                    return;
                }
                else if(field_metadata.field_status != VALID || field_metadata.raw_field_length != 9){
                    // If the field is not valid,  but also not en_route, just considere it as unavailable and continue parsing the next field
                    m10_gnss_module->time_of_sample.is_available = 0;
                    break;
                }

                NmeaParseUtcTime(&(m10_gnss_module->time_of_sample), &raw_field_data);
                break;

            case 1:
                break;

            case 2:
                if(field_metadata.field_status != VALID || field_metadata.raw_field_length != 10){
                    m10_gnss_module->latitude.is_available = 0;
                    break;
                }

                NmeaParseLatLong(&(m10_gnss_module->latitude), &raw_field_data, LATITUDE);
                m10_gnss_module->latitude.is_available = 1;
                break;

            case 3:
                if(field_metadata.field_status != VALID || field_metadata.raw_field_length != 1){
                    break;
                }

                m10_gnss_module->latitude.indicator = raw_field_data[0];
                break;

            case 4:
                if(field_metadata.field_status != VALID || field_metadata.raw_field_length != 11){
                    m10_gnss_module->longitude.is_available = 0;
                    break;
                }

                NmeaParseLatLong(&(m10_gnss_module->longitude), &raw_field_data, LONGITUDE);
                m10_gnss_module->longitude.is_available = 1;
                break;

            case 5:
                if(field_metadata.field_status != VALID || field_metadata.raw_field_length != 1){
                    break;
                }

                m10_gnss_module->longitude.indicator = raw_field_data[0];
                break;

            case 6:
                if(field_metadata.field_status != VALID){
                    m10_gnss_module->speed_over_ground_knots.is_available = 0;
                    break;
                }

                m10_gnss_module->speed_over_ground_knots.value = NmeaParseNumericFloatingPoint(&raw_field_data);
                m10_gnss_module->speed_over_ground_knots.is_available = 1;
                break;

            case 7:
                if(field_metadata.field_status != VALID){
                    m10_gnss_module->course_over_ground.is_available = 0;
                    break;
                }

                m10_gnss_module->course_over_ground.value = NmeaParseNumericFloatingPoint(&raw_field_data);
                m10_gnss_module->course_over_ground.is_available = 1;
                break;

            case 8:
                if(field_metadata.field_status != VALID || field_metadata.raw_field_length != 6){
                    break;
                }

                NmeaParseUtcDate(&(m10_gnss_module->time_of_sample), &raw_field_data);
                break;

            default:
                break;
        }
    
        field_index++;

        if(field_metadata.field_status == END_OF_MESSAGE){
            field_index = 0;
            raw_stream_buffer_parser_state = IDLE;
            return;
        }
    
    }
    
}

void M10GnssDriverGsvParser(nmea_caller_id* nmea_origin_id){
}
