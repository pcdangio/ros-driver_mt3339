/// \file message.h
/// \brief Defines the message class.
#ifndef MESSAGE_H
#define MESSAGE_H

#include <vector>
#include <string>

///
/// \brief A class representing a NMEA string message.
///
class message
{
public:
    // CONSTRUCTORS
    ///
    /// \brief message Instantiates a new message.
    /// \param talker The talker of the message.
    /// \param type The message type.
    ///
    message(std::string talker, std::string type);
    ///
    /// \brief message Instantiates a new message with data fields.
    /// \param talker The talker of the message.
    /// \param type The message type.
    /// \param data The data fields of the message.
    ///
    message(std::string talker, std::string type, std::vector<std::string> data);
    ///
    /// \brief message Instantiates a new message from a received NMEA string.
    /// \param nmea_sentence The received NMEA string.
    ///
    message(std::string nmea_sentence);

    // METHODS
    ///
    /// \brief validate_checksum Validates the checksum of the message.
    /// \return Returns TRUE if the message's checksum is valid, otherwise FALSE.
    ///
    bool validate_checksum() const;

    // PROPERTIES
    ///
    /// \brief p_talker Gets the talker of the message.
    /// \return The talker of the message.
    ///
    std::string p_talker() const;
    ///
    /// \brief p_type Gets the message type.
    /// \return The message type.
    ///
    std::string p_type() const;
    ///
    /// \brief p_data Gets the data of the message.
    /// \return The message data as a vector.
    ///
    std::vector<std::string> p_data() const;
    ///
    /// \brief p_nmea_sentence Gets the NMEA sentence of the message.
    /// \return The NMEA sentence of the message.
    ///
    std::string p_nmea_sentence() const;

private:
    // VARIABLES
    ///
    /// \brief m_sentence Stores the message in full NMEA sentence form.
    ///
    std::string m_sentence;

    // METHODS
    ///
    /// \brief construct Constructs a new message.
    /// \param talker The message talker.
    /// \param type The message type.
    /// \param data The message data.
    ///
    void construct(std::string talker, std::string type, std::vector<std::string> data);
    ///
    /// \brief calculate_checksum Calculates the checksum of an NMEA sentence.
    /// \param nmea_sentence The NMEA sentence to calculate the checksum for.
    /// \return The NMEA-compliant checksum string of the NMEA sentence.
    ///
    static std::string calculate_checksum(std::string nmea_sentence);
};

#endif // MESSAGE_H
