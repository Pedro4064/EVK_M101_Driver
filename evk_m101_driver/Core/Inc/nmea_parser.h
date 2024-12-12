#include "m10gnss_driver.h"

#ifndef __NMEA_PARSER_H__
#define __NMEA_PARSER_H__

#define NMEA_CALLER_ID_SIZE 5    // 4 (size of Address Field in NMEA standard) + 1 (\0)
#define NMEA_RAW_BUFFER_SIZE 20  // Max number of characters in an NMEA field

/**
 * @brief Type definition for the nmea message origin ID, also
 * known as the "Address Field" in the NMEA frame structure, comprised 
 * of the `Talker Identifier (TT)` (constellation of origin) and the 
 * `Sentence FormatterSSS` (the type of message): `TTSSS`.
 *    For more information: 
 * https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf
 * 
 */
typedef unsigned char nmea_caller_id[6];

/**
 * @brief Possible Status returned by the NMEA parser referring to a single filed
 * int the message.
 * 
 */
typedef enum NMEA_RAW_FIELD_STATUS{
    VALID,           // Valid field with non null data
    EMPTY,           // Filed empty with null data
    END_OF_MESSAGE,  // Message that ended the message frame
    PARSING_EN_ROUTE // Parsing occurring but could not be completed due to message slicing
}nmea_raw_field_status;

/**
 * @brief Enum to differentiate to if the user wants the parser to read a lat or long value, 
 * since the have different structures.
 * 
 */
typedef enum NMEA_LAT_LONG_PARSER{
    LATITUDE,
    LONGITUDE
}nmea_lat_long_parser;

/**
 * @brief The metadata from the parsing of an NMEA field.
 * 
 */
typedef struct NMEA_RAW_FIELD_METADATA{
    unsigned char raw_field_length;     // Number of characters in the NMEA field
    nmea_raw_field_status field_status; // Status of the field parsing
}nmea_raw_field_metadata;

/**
 * @brief Compare two address fields with the possibility of the `*` wildcard, for example:
 * @code
 * nmea_caller_id caller_1 = "GPRMC"; // With specific talker/constellation id
 * nmea_caller_id caller_2 = "**RMC"; // Talker not of interest
 * char comparison_result = 0;
 * 
 * comparison_result = NmeaParserCompareOriginId(caller_1, caller_2); // Returns 1
 * @endcode
 * 
 * @param message_origin: `nmea_caller_id*` Pointer to the first address field
 * @param table_origin: `nmea_caller_id*` Pointer to the second address field
 * @return char: `1` if equal (considering wild cards) and `0` otherwise
 */
char NmeaParserCompareOriginId(nmea_caller_id* message_origin, nmea_caller_id* table_origin);

/**
 * @brief Get the next `,` delimited filed in the NMEA message. If the message was cut (message slicing)
 * due to buffer limitations, the metadata will return `PARSING_EN_ROUTE` for the `nmea_raw_field_status`, 
 * and in the next call it will resume the parsing.
 * 
 * @param stream_buffer: `m10_gnss_stream_buffer*` Pointer to the buffer struct containing both the buffer and
 * necessary metadata for parsing.
 * @param raw_field_buffer: `char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]` Pointer an array of NMEA_RAW_BUFFER_SIZE
 * number of characters, to hold the extracted field's characters.
 * @return nmea_raw_field_metadata: `nmea_raw_field_metadata` Metadata containing parsing results and field length.
 */
nmea_raw_field_metadata NmeaGetNextFieldRaw(m10_gnss_stream_buffer* stream_buffer, char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]);

/**
 * @brief Parse the raw field characters into UTC formatted time.
 * 
 * @param date_time: `utc_date_time*` Pointer to instance of utc date time to hold final value
 * @param raw_field_buffer: `char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]` Pointer an array of NMEA_RAW_BUFFER_SIZE
 * number of characters, to hold the extracted field's characters.
 */
void NmeaParseUtcTime(utc_date_time* date_time, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]);

/**
 * @brief Parse the raw field characters into UTC formatted date.
 * 
 * @param date_time: `utc_date_time*` Pointer to instance of utc date time to hold final value
 * @param raw_field_buffer: `char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]` Pointer an array of NMEA_RAW_BUFFER_SIZE
 * number of characters, to hold the extracted field's characters.
 */
void NmeaParseUtcDate(utc_date_time* date_time, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]);

/**
 * @brief Parse the raw field characters into latitude or longitude format.
 * 
 * @param lat_long_measurement: `gnss_lat_long_measurement*` Pointer to the instance of lat_long_measurement to hold final value
 * @param raw_field_buffer: `char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]` Pointer an array of NMEA_RAW_BUFFER_SIZE
 * number of characters, to hold the extracted field's characters.
 * @param nmea_parser_option: `nmea_lat_long_parser` Specify rather to parse data as a latitude field or longitude field
 */
void NmeaParseLatLong(gnss_lat_long_measurement* lat_long_measurement, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE], nmea_lat_long_parser nmea_parser_option);

/**
 * @brief Parse and convert field data to double. 
 * 
 * @param raw_field_buffer: `char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]` Pointer an array of NMEA_RAW_BUFFER_SIZE
 * number of characters, to hold the extracted field's characters.
 * @return double: Converted value
 */
double NmeaParseNumericFloatingPoint(char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]);
#endif