#include "m10gnss_driver.h"

#ifndef __NMEA_PARSER_H__
#define __NMEA_PARSER_H__

#define NMEA_CALLER_ID_SIZE 5
#define NMEA_RAW_BUFFER_SIZE 20

typedef unsigned char nmea_caller_id[6];
typedef enum NMEA_RAW_FIELD_STATUS{
    VALID,
    EMPTY,
    END_OF_MESSAGE,
    PARSING_EN_ROUTE
}nmea_raw_field_status;

typedef enum NMEA_LAT_LONG_PARSER{
    LATITUDE,
    LONGITUDE
}nmea_lat_long_parser;

typedef struct NMEA_RAW_FIELD_METADATA{
    unsigned char raw_field_length;
    nmea_raw_field_status field_status;
}nmea_raw_field_metadata;

char NmeaParserCompareOriginId(nmea_caller_id* message_origin, nmea_caller_id* table_origin);

nmea_raw_field_metadata NmeaGetNextFieldRaw(m10_gnss_stream_buffer* stream_buffer, char (*raw_field_buffer)[NMEA_RAW_BUFFER_SIZE]);

void NmeaParseUtcTime(utc_date_time* date_time, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]);

void NmeaParseUtcDate(utc_date_time* date_time, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]);

void NmeaParseLatLong(gnss_lat_long_measurement* lat_long_measurement, char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE], nmea_lat_long_parser nmea_parser_option);

double NmeaParseNumericFloatingPoint(char (*raw_stream_buffer)[NMEA_RAW_BUFFER_SIZE]);
#endif