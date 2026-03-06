/*
 * nmea_parser.c - NMEA Sentence Parser
 */

#include "nmea_parser.h"
#include <string.h>
#include <stdlib.h>

/* ── Internal helpers ──────────────────────────────────────────────── */

/**
 * Find the start index of the last $G?GGA sentence in the buffer.
 * Returns -1 if not found.
 */
static int find_last_gga(const uint8_t *buf, size_t len)
{
    if (len < 60) return -1;

    int last = -1;
    for (size_t i = 0; i + 5 < len; i++) {
        if (buf[i] == '$' && buf[i + 1] == 'G' &&
            (buf[i + 2] == 'N' || buf[i + 2] == 'P') &&
            buf[i + 3] == 'G' && buf[i + 4] == 'G' && buf[i + 5] == 'A') {
            last = (int)i;
        }
    }
    return last;
}

/**
 * Split a single NMEA sentence into comma-separated fields.
 * Returns the number of fields parsed.
 */
#define MAX_FIELDS 15
#define MAX_FIELD_LEN 20

static int split_fields(const uint8_t *sentence, size_t max_scan,
                        char fields[][MAX_FIELD_LEN])
{
    int field = 0;
    int pos = 0;

    for (size_t i = 0; i < max_scan && field < MAX_FIELDS; i++) {
        uint8_t c = sentence[i];
        if (c == '\n' || c == '\r' || c == '*') break;

        if (c == ',') {
            fields[field][pos] = '\0';
            field++;
            pos = 0;
        } else if (c != '$' && pos < MAX_FIELD_LEN - 1) {
            fields[field][pos++] = (char)c;
        }
    }
    fields[field][pos] = '\0';
    return field + 1;
}

/**
 * Convert NMEA coordinate (DDMM.MMMM or DDDMM.MMMM) to decimal degrees.
 */
static double nmea_coord_to_degrees(const char *raw, char hemisphere)
{
    double val = atof(raw);
    int degrees = (int)(val / 100);
    double minutes = val - (degrees * 100);
    double result = degrees + (minutes / 60.0);

    if (hemisphere == 'S' || hemisphere == 'W') {
        result = -result;
    }
    return result;
}

/* ── Public API ────────────────────────────────────────────────────── */

nmea_gga_data_t nmea_parse_gga(const uint8_t *buf, size_t len)
{
    nmea_gga_data_t result = { .valid = false };

    int gga_start = find_last_gga(buf, len);
    if (gga_start < 0) return result;

    /* Parse fields from the GGA sentence */
    char fields[MAX_FIELDS][MAX_FIELD_LEN] = {{0}};
    size_t remaining = len - gga_start;
    if (remaining > 200) remaining = 200;

    int nfields = split_fields(buf + gga_start, remaining, fields);

    /*
     * GGA field layout:
     *   [0] GNGGA   [1] time      [2] lat       [3] N/S
     *   [4] lon     [5] E/W       [6] quality   [7] num sats
     *   [8] HDOP    [9] altitude  [10] alt unit  [11] geoid sep ...
     */
    if (nfields < 10) return result;
    if (strlen(fields[2]) == 0 || strlen(fields[4]) == 0) return result;

    result.latitude   = nmea_coord_to_degrees(fields[2], fields[3][0]);
    result.longitude  = nmea_coord_to_degrees(fields[4], fields[5][0]);
    result.altitude   = atof(fields[9]);
    result.satellites = atoi(fields[7]);
    result.fix_quality = atoi(fields[6]);
    result.valid      = true;

    return result;
}

const char *nmea_fix_quality_str(int quality)
{
    switch (quality) {
        case FIX_RTK_FIXED:
            return "RTK FIXED (~1-3 cm, best precision)";
        case FIX_RTK_FLOAT:
            return "RTK FLOAT (~0.2-1 m, RTK converging)";
        case FIX_DGPS:
            return "DGPS/SBAS (~0.5-3 m, corrected)";
        case FIX_GPS:
            return "GPS SPS (~3-10 m, standalone)";
        case FIX_NONE:
            return "No Fix (position not reliable)";
        default:
            return "Unknown Fix";
    }
}
