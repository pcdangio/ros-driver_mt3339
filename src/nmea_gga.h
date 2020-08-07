#ifndef NMEA___GGA_H
#define NMEA___GGA_H

#include <stdint.h>

namespace nmea
{

struct gga
{
    double utc_time_of_day;
    double latitude;
    double longitude;
    enum class fix_type_t
    {
        NONE = 0,
        GPS = 1,
        DGPS = 2
    } fix_type;
    uint8_t satellite_count;
    double hdop;
    double altitude;
    double geoid_height;
    double dgps_age;
    uint32_t dgps_id;
};

}

#endif