#include "message.h"

#include <sstream>
#include <iomanip>

// CONSTRUCTORS
message::message(std::string talker, std::string type)
{
    message::construct(talker, type, std::vector<std::string>());
}
message::message(std::string talker, std::string type, std::vector<std::string> data)
{
    message::construct(talker, type, data);
}
message::message(std::string nmea_sentence)
{
    message::m_sentence = nmea_sentence;
}

// METHODS
void message::construct(std::string talker, std::string type, std::vector<std::string> data)
{
    // Create stringstream for generating string.
    std::stringstream stream;

    // Set header, talker, and message type.
    stream << '$'
           << talker
           << type;
    // Add in data fields.
    for(unsigned int d = 0; d < data.size(); d++)
    {
        stream << ','
               << data.at(d);
    }
    // Add in data end flag.
    stream << '*';
    // Add in checksum, calculated on current stream.
    stream << message::calculate_checksum(stream.str());
    // Add CRLF footer.
    stream << "\r\n";

    // Store sentence.
    message::m_sentence = stream.str();
}
std::string message::calculate_checksum(std::string nmea_sentence)
{
    // Find location of *
    unsigned long end_position = nmea_sentence.find_last_of('*');

    // Checksum is XOR of everything between $ and *
    unsigned char checksum = 0;
    for(unsigned long i = 1; i < end_position; i++)
    {
        checksum ^= nmea_sentence.at(i);
    }

    // Convert checksum into hex string.
    std::stringstream stream;
    stream << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(checksum);

    // Return hex string.
    return stream.str();
}
bool message::validate_checksum() const
{
    // Pull actual checksum substring.
    // First find *
    unsigned long position = message::m_sentence.find_last_of('*');
    if(position == std::string::npos || position + 2 >= message::m_sentence.size())
    {
        // Malformed packet.
        return false;
    }
    // Substring the actual checksum.
    std::string actual = message::m_sentence.substr(position + 1, 2);

    // Calculate the checksum of the sentence.
    std::string calculated = message::calculate_checksum(message::m_sentence);

    // Return the comparison.
    return (actual.compare(calculated) == 0);
}

// PROPERTIES
std::string message::p_talker() const
{
    // Find index of either first , or last *
    unsigned long end_position;
    if((end_position = message::m_sentence.find_first_of(',')) == std::string::npos)
    {
        end_position = message::m_sentence.find_last_of('*');
    }

    // Calculate talker field length:
    unsigned long field_length = end_position - 4;

    // Return talker substring.
    return message::m_sentence.substr(1, field_length);
}
std::string message::p_type() const
{
    // Find index of either first , or last *
    unsigned long end_position;
    if((end_position = message::m_sentence.find_first_of(',')) == std::string::npos)
    {
        end_position = message::m_sentence.find_last_of('*');
    }

    // Calculate start position.
    unsigned long start_position = end_position - 3;

    // Return talker substring.
    return message::m_sentence.substr(start_position, 3);
}
std::vector<std::string> message::p_data() const
{
    // Create output vector.
    std::vector<std::string> output;

    // Search for all commas.
    unsigned long position = 0;
    while((position = message::m_sentence.find(',', position)) != std::string::npos)
    {
        // Comma found.

        // First increment position to point to first character in this field
        position++;

        // Find end of this field, which is marked by , or *
        unsigned long end_position;
        if((end_position = message::m_sentence.find_first_of(',', position)) == std::string::npos)
        {
            end_position = message::m_sentence.find_last_of('*');
        }

        // Calculate field length.
        unsigned long field_length = end_position - position;

        // Add substring to output.
        output.push_back(message::m_sentence.substr(position, field_length));
    }

    // Return output vector.
    return output;
}
std::string message::p_nmea_sentence() const
{
    return message::m_sentence;
}
