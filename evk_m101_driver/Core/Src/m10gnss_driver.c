#include "i2c.h"
#include "m10gnss_driver.h"
#include "nmea_parser.h"

#define AVAILABLE_BUFFER_HB 0xFD
#define AVAILABLE_BUFFER_LB 0xFE

#define MESSAGE_START '$'
#define NUM_PARSING_TABLE_ENTRIES 2

/**
 * @internal
 * @brief Entry for a caller lookup table, which relates the constellation that
 * generated the message and type of message (called `nmea_caller_id`) and the 
 * callback to parse the message in the NMEA format.
 *    The `nmea_caller_id` supports wildcard  `*` for characters that do not matter 
 *  for message -> parsing function matching, for example: 
 * @code
 *       `... = {
 *        {...},
 *        {
 *         .message_origin = "**GSV",
 *         .parser_function = M10GnssDriverGsvParser
 *        }`
 *    
 * @endcode
 *    In the above example, it does not matter the constellation of origin, all `GSV` messages would be parsed 
 * by corresponding parsing function `M10GnssDriverGsvParser`.
 *    For more information about possible constellations and all the message types supported by NMEA, check:
 * https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf
 *
 * @endinternal
 */
typedef struct NMEA_MESSAGE_PARSING_TABLE_ENTRY{
    nmea_caller_id message_origin;              // Constellation + message type, with possible `*` wild card
    void(*parser_function)(nmea_caller_id*) ;   // Callback to parse the message
} nmea_message_parsing_table_entry;

/**
 * @internal
 * @brief State of the parser, used when message slicing happens (i.e a message is broken due to buffer size
 * limitations).
 * 
 * @endinternal
 */
typedef enum PARSER_STATE{
    IDLE,
    PARSING, 
    DISCARDING_MESSAGE
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

/**
 * @internal 
 * @brief Initialize the M10 GNSS Driver
 * 
 * @param m10_module: `m10_gnss*` Pointer to an instance of m10_gnss
 * 
 *    Initializes the Driver by saving the pointer to the m10_gnss instance containing all the 
 * necessary files and the handler for the I2C com.
 *    Furthermore clears all the buffer from the Ublox module by reading it until empty, as to avoid 
 * computing old data.
 * @endinternal 
 */
void M10GnssDriverInit(m10_gnss* m10_module){
    m10_gnss_module = m10_module;
    M10GnssDriverClearStreamBuffer();
}

/**
 * @internal 
 * @brief Discard the message in the local buffer by incrementing the buffer index until a new line character \\n
 * is met.
 *    If the message was cut short (i.e the index reaches the last position before the new line character is found)
 * the system will go into the DISCARDING_MESSAGE mode and return, this way it ensures that after the next buffer read
 * it will resume here.
 * @endinternal 
 */
void M10GnssDriverNmeaDiscardMessage(void){
    // Set the status to discarding, so in case the buffer ends the next read will start here to end the discarding
    raw_stream_buffer_parser_state = DISCARDING_MESSAGE;
    while(raw_stream_buffer.buffer_index < raw_stream_buffer.buffer_size){

        // if the end of message character is found, break from the loop and set the state back to idle
        // so the next message can be parsed
        if(raw_stream_buffer.buffer[raw_stream_buffer.buffer_index] == '\n')
            break;

        raw_stream_buffer.buffer_index++;
    }

    raw_stream_buffer_parser_state = IDLE;
}

/**
 * @internal 
 * @brief From the caller ID, delegates the parsing to the correct parsing function, established in the `nmea_message_parsing_table`
 * lookup table, with the ability to parse `*` wildcard for characters that do not matter for message -> parsing function matching 
 * 
 * @param nmea_origin_id: `nmea_caller_id*` Pointer to the nmea caller id of the message.
 * @endinternal 
 */
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

/**
 * @internal 
 * @brief Gets the number of bytes in the module's stream buffer.
 * 
 * @return uint16_t Number of bytes to be read in the buffer
 * @endinternal 
 */
uint16_t M10GnssDriverGetStreamBufferSize(void){
        
        unsigned char raw_buffer_val;
        uint16_t buffer_size = 0;

        HAL_I2C_Mem_Read(m10_gnss_module->i2c_handle, m10_gnss_module->i2c_address, AVAILABLE_BUFFER_HB, STREAM_BUFFER_REGISTER_SIZE, &raw_buffer_val, 1, 10000);
        buffer_size |= raw_buffer_val<<4;

        HAL_I2C_Mem_Read(m10_gnss_module->i2c_handle, m10_gnss_module->i2c_address, AVAILABLE_BUFFER_LB, STREAM_BUFFER_REGISTER_SIZE, &raw_buffer_val, 1, 10000);
        buffer_size |= raw_buffer_val;
        return buffer_size;
}

/**
 * @internal 
 * @brief Read the data in the module's stream buffer through I2C.
 *    The Maximum number of bytes to be read at one time is 400, to change this limit
 * it is necessary to change the size of STACK_BUFFER_ARRAY_SIZE, although it is necessary to 
 * be mindful of the available stack size.
 * 
 * @endinternal 
 */
void M10GnssDriverReadStreamBuffer(void){

        raw_stream_buffer.buffer_size = M10GnssDriverGetStreamBufferSize();
        raw_stream_buffer.buffer_size = (raw_stream_buffer.buffer_size > STACK_BUFFER_ARRAY_SIZE)?STACK_BUFFER_ARRAY_SIZE:raw_stream_buffer.buffer_size;
        HAL_I2C_Mem_Read(m10_gnss_module->i2c_handle, m10_gnss_module->i2c_address, STREAM_BUFFER_REGISTER, STREAM_BUFFER_REGISTER_SIZE, &raw_stream_buffer.buffer, raw_stream_buffer.buffer_size, 10000);
        raw_stream_buffer.buffer_index = 0;
}

/**
 * @internal 
 * @brief Clear the module's stream buffer.
 *    It tries a maximum of 50 times to clear the buffer or until the stream buffer size is reported as 0
 * by the module. This limit is a way to avoid getting stuck if, by any chance, the buffer is being filled
 * faster than we are able to clear it.
 * 
 * @endinternal 
 */
void M10GnssDriverClearStreamBuffer(void){

    for (int i = 0; i < 50; i++){
        M10GnssDriverReadStreamBuffer();
        if(raw_stream_buffer.buffer_size == 0)
            return;
    }
    
}

/**
 * @internal 
 * @brief Parse a new message by first parsing the caller id (first 5 characters) and calling the message delegator.
 *    If the message was cut in the middle of its caller id due to buffer size limits, it keeps track of the caller id
 * index (used to construct a local copy of the caller id to latter be compared by the Message Delegator), and by that 
 * in the next parsing iteration (after the buffer read call) it resumes parsing the caller id.
 * 
 * @endinternal 
 */
void M10GnssDriverParseNewMessage(void){
    static int nmea_caller_id_index;
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

/**
 * @internal 
 * @brief Parse the data streamed from the module.
 *    This function is also responsible to resume the correct parsing context based on the termination status of the  
 * previous iteration, i.e if the last buffer was complete, it will start parsing a new message. If the last buffer  
 * was already parsing a message, it will call the message delegator (hence the importance off the message parsing
 * functions to be able to retain context in between calls in case of message spliting). And finally if a message 
 * was being discarded (because no callback is listed) it shall call `M10GnssDriverNemeaDiscardMessage` at once.
 * 
 * @endinternal 
 */
void M10GnssDriverParseBuffer(void){

    while(raw_stream_buffer.buffer_index < raw_stream_buffer.buffer_size){

        switch (raw_stream_buffer_parser_state){

            case IDLE:
                M10GnssDriverParseNewMessage();
                break;

            case PARSING:
                M10GnssDriverNmeaMessageDelegator(message_origin);
                break;
        
            case DISCARDING_MESSAGE:
                M10GnssDriverNmeaDiscardMessage();
                break;

            default:
                break;
        }

        
    }
    
}

/**
 * @internal 
 * @brief Read and parse the data on the module's stream buffer.
 * 
 * @endinternal 
 */
void M10GnssDriverReadData(void){

    M10GnssDriverReadStreamBuffer();

    if(raw_stream_buffer.buffer_size == 0)
        return;

    // If the first element is $, force the state back to idle, to avoid parsing error propagation
    if(raw_stream_buffer.buffer[0] == '$')
        raw_stream_buffer_parser_state = IDLE;

    M10GnssDriverParseBuffer();
    
}

/**
 * @internal
 * @brief Parses NMEA messages of type RMC (Recommended minimum data), as described in the 
 * user's manual: https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf
 * 
 * @param nmea_origin_id: `nmea_caller_id*` pointer to the caller id (i.e the constellation) that generated the message.
 * @endinternal
 */
void M10GnssDriverRmcParser(nmea_caller_id* nmea_origin_id){

    static char raw_field_data[20];           // Buffer containing the raw NMEA field
    static int field_index;                   // Index of the field being parsed at the moment
    
    raw_stream_buffer_parser_state = PARSING; // Set the parser state to PARSING, so if message is cut due to buffer limit, resume parsing here

    while(field_index < 15){

        // If parsing en route but the message was cut due to buffer size constraints, just return to ParseBuffer function with the
        // parser state still as PARSING, and field index as 0, so it will continue the parsing here
        nmea_raw_field_metadata field_metadata = NmeaGetNextFieldRaw(&raw_stream_buffer, &raw_field_data);
        if(field_metadata.field_status == PARSING_EN_ROUTE)
                    return;

        switch (field_index){

            case 0:
                
                if(field_metadata.field_status != VALID || field_metadata.raw_field_length != 9){
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
