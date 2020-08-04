#ifndef NMEA___GSA_H
#define NMEA___GSA_H

#include <stdint.h>
#include <vector>

namespace nmea
{

struct gsa
{
    enum class mode_t
    {
        MANUAL = 0,
        AUTOMATIC = 1
    } mode;
    enum class fix_type_t
    {
        NONE = 0,
        _2D = 1,
        _3D = 2
    } fix_type;
    std::vector<uint16_t> satellites;
    float pdop;
    float hdop;
    float vdop;
};

}

#endif