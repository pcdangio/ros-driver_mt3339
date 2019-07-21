#ifndef MESSAGE_H
#define MESSAGE_H

#include <vector>
#include <string>

class message
{
public:
    enum class id
    {
        PMTK_ACK =                  001,
        PMTK_SYS_MSG =              010,
        PMTK_SET_NMEA_UPDATERATE =  220,
        PMTK_SET_NMEA_BAUDRATE =    251,
        PMTK_SET_API_NMEA_OUTPUT =  314
    };
    message(id command_id);
    message(std::string serialized);

    void add_field(int data);

    std::string serialize();

private:
    id m_id;
    std::vector<std::string> m_data;


};

#endif // MESSAGE_H
