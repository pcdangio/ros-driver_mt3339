#ifndef DRIVER_H
#define DRIVER_H

#include <serial/serial.h>
#include "message.h"

#include <map>
#include <ctime>
#include <functional>

class driver
{
public:
    enum class baud_rate
    {
        bps_4800 = 4800,
        bps_9600 = 9600,
        bps_14400 = 14400,
        bps_19200 = 19200,
        bps_38400 = 38400,
        bps_57600 = 57600,
        bps_115200 = 115200
    };
    enum class nmea_type
    {
        GLL = 0,
        RMC = 1,
        VTG = 2,
        GGA = 3,
        GSA = 4,
        GSV = 5,
        ZDA = 17,
        CHN = 18
    };
    enum class nmea_rate
    {
        DISABLED = 0,
        FIX_1 = 1,
        FIX_2 = 2,
        FIX_3 = 3,
        FIX_4 = 4,
        FIX_5 = 5
    };

    struct data
    {
        enum class fix_type
        {
            NONE = 1,
            FIX_2D = 2,
            FIX_3D = 3
        };

        std::time_t time;
        fix_type fix;

        float latitude;
        float longitude;
        float altitude;
        float hdop;
        float vdop;
    };

    driver(std::string port, unsigned int baud_rate);
    ~driver();

    bool set_baud(unsigned int baud_rate);
    bool set_nmea_update_rate(unsigned int milliseconds);
    bool set_nmea_output(std::map<nmea_type, nmea_rate> outputs);

    void attach_data_callback(std::function<void(data)> callback);

    void spin();

private:
    serial::Serial* m_port;
    data m_current_data;
    std::function<void(data)> m_callback;

    bool send(message msg);

    void write(message msg);
    message* read(std::string talker, std::string type);


};

#endif // DRIVER_H
