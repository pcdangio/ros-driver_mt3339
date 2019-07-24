#ifndef MESSAGE_H
#define MESSAGE_H

#include <vector>
#include <string>

class message
{
public:
    enum class talker
    {
        UNKNOWN = 0,
        PMTK = 1,
        NMEA = 2
    };
    enum class id
    {
        UNKNOWN =                   000,
        PMTK_ACK =                  001,
        PMTK_SYS_MSG =              010,
        PMTK_TXT_MSG =              011,
        PMTK_SET_NMEA_UPDATERATE =  220,
        PMTK_SET_NMEA_BAUDRATE =    251,
        PMTK_SET_API_NMEA_OUTPUT =  314
    };


    message(id command_id);
    message(std::string serialized);

    void add_field(unsigned int data);
    unsigned int get_field(unsigned int address) const;

    std::string serialize() const;


    talker p_talker() const;
    id p_id() const;

private:
    talker m_talker;
    id m_id;
    std::vector<std::string> m_data;

    void parse_pmtk(std::string serialized);
    void parse_nmea(std::string serialized);

    std::string calculate_checksum() const;
    bool validate_checksum() const;
};

#endif // MESSAGE_H
