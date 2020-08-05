#include "nmea_sentence.h"

#include <sstream>
#include <iomanip>

using namespace nmea;

// CONSTRUCTORS
sentence::sentence(const std::string& talker, const std::string& type, uint32_t n_fields)
{
    // Store the talker.
    sentence::m_talker = talker;
    // Store the type.
    sentence::m_type = type;
    // Initialize the fields.
    sentence::m_fields.resize(n_fields);
}
sentence::sentence(const std::string& nmea_sentence)
{
    // Strip out the checksum if it has one.
    std::stringstream parser;
    if(nmea_sentence.at(nmea_sentence.length() - 5) == '*')
    {
        parser << nmea_sentence.substr(0, nmea_sentence.length() - 5);
    }
    else
    {
        parser << nmea_sentence;
    }
    
    // Create a stringstream for parsing.
    std::string part;

    // Parse the talker and type.
    std::getline(parser, part, ',');
    // Assume talker length is variable and type length is 3.
    sentence::m_talker = part.substr(1, part.length() - 4);
    sentence::m_type = part.substr(part.length() - 3);

    // Parse the fields.
    while(std::getline(parser, part, ','))
    {
        sentence::m_fields.push_back(part);
    }
}

// METHODS
std::string sentence::calculate_checksum(std::string nmea_sentence)
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
bool sentence::validate_checksum(const std::string& nmea_sentence)
{
    // Pull actual checksum substring.
    // First find *
    unsigned long position = nmea_sentence.find_last_of('*');
    if(position == std::string::npos || position + 2 >= nmea_sentence.size())
    {
        // Malformed packet.
        return false;
    }
    // Substring the actual checksum.
    std::string actual = nmea_sentence.substr(position + 1, 2);

    // Compare the actual to the calculated checksum.
    return actual == sentence::calculate_checksum(nmea_sentence);
}

// PROPERTIES
std::string sentence::talker() const
{
    return sentence::m_talker;
}
std::string sentence::type() const
{
    return sentence::m_type;
}
uint32_t sentence::n_fields() const
{
    return sentence::m_fields.size();
}
bool sentence::has_field(uint32_t i) const
{
    // Check if index is valid.
    if(i < sentence::m_fields.size())
    {
        // Check if field is not empty.
        return !sentence::m_fields.at(i).empty();
    }
    else
    {
        return false;
    }
}
std::string sentence::get_field(uint32_t i) const
{
    // Check if the index is valud.
    if(i < sentence::m_fields.size())
    {
        return sentence::m_fields.at(i);
    }
    else
    {
        return "";
    }
}
void sentence::set_field(uint32_t i, const std::string& value)
{
    // Check if index is valid.
    if(i < sentence::m_fields.size())
    {
        sentence::m_fields[i] = value;
    }
}
std::string sentence::p_nmea_sentence() const
{
    // Create stringstream for generating string.
    std::stringstream stream;

    // Set header, talker, and sentence type.
    stream << '$'
           << sentence::m_talker
           << sentence::m_type;
    // Add in data fields.
    for(auto field = sentence::m_fields.cbegin(); field != sentence::m_fields.cend(); ++ field)
    {
        stream << ',' << *field;
    }
    // Add in data end flag.
    stream << '*';
    // Add in checksum, calculated on current stream.
    stream << sentence::calculate_checksum(stream.str());
    // Add CRLF footer.
    stream << "\r\n";

    // Return the sentence.
    return stream.str();
}
