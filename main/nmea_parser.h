/*
 * nmea_parser.h - NMEA Sentence Parser
 *
 * Parses NMEA GGA sentences from the ZED-F9P to extract
 * position, altitude, satellite count, and RTK fix quality.
 */

#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* RTK fix quality codes (from NMEA GGA field 6) */
typedef enum {
    FIX_NONE      = 0,
    FIX_GPS       = 1,
    FIX_DGPS      = 2,
    FIX_RTK_FIXED = 4,
    FIX_RTK_FLOAT = 5,
} nmea_fix_quality_t;

/* Parsed GPS position data */
typedef struct {
    double   latitude;          /* Decimal degrees (negative = South) */
    double   longitude;         /* Decimal degrees (negative = West)  */
    double   altitude;          /* Meters above MSL */
    int      satellites;        /* Number of satellites in use */
    int      fix_quality;       /* One of nmea_fix_quality_t values */
    bool     valid;             /* True if a GGA sentence was found & parsed */
} nmea_gga_data_t;

/**
 * Parse the most recent GGA sentence from a raw NMEA byte buffer.
 *
 * Searches backwards for the last complete $G?GGA sentence and
 * extracts position, altitude, satellite count, and fix quality.
 *
 * @param buf  Raw byte buffer containing NMEA sentences
 * @param len  Length of the buffer
 * @return     Parsed GGA data (check .valid before using)
 */
nmea_gga_data_t nmea_parse_gga(const uint8_t *buf, size_t len);

/**
 * Return a human-readable string for a fix quality value.
 */
const char *nmea_fix_quality_str(int quality);

#endif /* NMEA_PARSER_H */
