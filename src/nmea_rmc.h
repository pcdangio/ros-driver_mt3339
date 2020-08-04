#ifndef NMEA___RMC_H
#define NMEA___RMC_H

#include <stdint.h>

namespace nmea
{

struct rmc
{
    double utc_time;
    enum class status_t
    {
        VOID = 0,
        ACTIVE = 1
    } status;
    double latitude;
    double longitude;
    double ground_speed;
    double track_true;
    double magnetic_variation;
    enum class mode_t
    {
        INVALID = 0,
        AUTONOMOUS = 1,
        DIFFERENTIAL = 2,
        ESTIMATED = 3,
        MANUAL = 4
    } mode;
};

}

#endif