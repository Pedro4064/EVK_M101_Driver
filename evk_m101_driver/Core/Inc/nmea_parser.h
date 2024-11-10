#include "m10gnss_driver.h"

#ifndef __NMEA_PARSER_H__
#define __NMEA_PARSER_H__

#define NMEA_CALLER_ID_SIZE 5
#define NMEA_RAW_BUFFER_SIZE 20

typedef unsigned char nmea_caller_id[6];

char NmeaParserCompareOriginId(nmea_caller_id* message_origin, nmea_caller_id* table_origin);

#endif