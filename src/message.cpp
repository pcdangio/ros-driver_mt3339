#include "message.h"

#include <sstream>
#include <iomanip>

// CONSTRUCTORS
message::message(id command_id)
{
    // Create empty message with specified ID.
    message::m_talker = message::talker::PMTK;
    message::m_id = command_id;
}
message::message(std::string serialized)
{
    // Deserialize message string.

    // Determine talker.
    // Check beginning of packet to determine talker.
    // Most messages will be $GP
    std::string sub_talker = serialized.substr(0, 3);
    if(sub_talker.compare("$GP") == 0)
    {
        message::m_talker = message::talker::NMEA;
    }

    std::string sub_talker = serialized.substr(1, 4);
    if(sub_talker.compare("PMTK") == 0)
    {
        message::m_talker = message::talker::PMTK;
    }
    else if(sub_talker.compare("NMEA") == 0)
    {
        message::m_talker = message::talker::NMEA;
    }
    else
    {
        message::m_talker = message::talker::UNKNOWN;
    }

    // Read command ID.
    try
    {
        message::m_id = static_cast<message::id>(std::stoi(serialized.substr(5, 3)));
    }
    catch(...)
    {
        message::m_id = message::id::UNKNOWN;
    }

    // Read data fields.
    unsigned long data_end = serialized.find('*');
    unsigned long data_length = data_end - 8;
    $PMTK000,1,2,3*
    01234567891

    if(data_end != 8)
    {
        // Data fields present.
        // Find location of data field end.
        std::string sub_data = serialized.substr(9, )
    }
    unsigned long position = serialized.find(',');
    while(position != std::string::npos)
    {
        unsigned long start = position;
        // Find the next comma position.
        position = serialized.find(',');
        // Extract current data field.
        message::m_data.push_back(serialized.substr(position, ))
    }
}

// METHODS
void message::add_field(unsigned int data)
{

}
unsigned int message::get_field(unsigned int address) const
{

}
std::string message::calculate_checksum() const
{

}
bool message::validate_checksum() const
{

}
std::string message::serialize() const
{
    // Serialize message into string.
    std::stringstream stream;
    // Add header.
    stream << '$';
    // Add talker type.
    switch(message::m_talker)
    {
    case message::talker::UNKNOWN:
    {
        stream << "UNKN";
        break;
    }
    case message::talker::NMEA:
    {
        stream << "NMEA";
        break;
    }
    case message::talker::PMTK:
    {
        stream << "PMTK";
        break;
    }
    }
    // Add message ID, with field width of 3.
    stream << std::setw(3) << std::setfill('0') << static_cast<unsigned int>(message::m_id);
    stream.copyfmt(std::ios(nullptr));
    // Add data fields.
    for(unsigned int n = 0; n < message::m_data.size(); n++)
    {
        stream << ',' << message::m_data.at(n);
    }
    // Add data end marker
    stream << '*';
    // Add checksum.
    stream << message::calculate_checksum();
    // Add footer.
    stream << "\r\n";

    return stream.str();
}

// PROPERTIES
message::talker message::p_talker() const
{
    return message::m_talker;
}
message::id message::p_id() const
{
    return message::m_id;
}
