/// \file nmea_sentence.h
/// \brief Defines the nmea::sentence class.
#ifndef NMEA___SENTENCE_H
#define NMEA___SENTENCE_H

#include <vector>
#include <string>

/// \brief Software related to the NMEA standard.
namespace nmea
{

/// \brief A class representing a NMEA sentence.
class sentence
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new sentence.
    /// \param talker The talker of the sentence.
    /// \param type The sentence type.
    /// \param n_fields The number of fields in the sentence.
    sentence(const std::string& talker, const std::string& type, uint32_t n_fields = 0);
    /// \brief Instantiates a new sentence from a received NMEA string.
    /// \param nmea_sentence The received NMEA string.
    sentence(const std::string& nmea_sentence);

    // METHODS
    /// \brief Validates the checksum of the sentence.
    /// \return Returns TRUE if the sentence's checksum is valid, otherwise FALSE.
    static bool validate_checksum(const std::string& nmea_sentence);

    // PROPERTIES
    /// \brief Gets the talker of the sentence.
    /// \return The talker of the sentence.
    std::string talker() const;
    /// \brief Gets the sentence type.
    /// \return The sentence type.
    std::string type() const;
    /// \brief Gets the number of fields in the sentence.
    /// \return The number of fields.
    uint32_t n_fields() const;
    /// \brief Checks if the sentence has a particular field.
    /// \param i The field to check.
    /// \return TRUE if the field exists, otherwise false.
    bool has_field(uint32_t i) const;
    /// \brief Gets a particular field from the sentence.
    /// \param i The index of the field to get.
    /// \return If the field exists, returns the field. Otherwise, returns an empty string.
    std::string get_field(uint32_t i) const;
    /// \brief Sets a particular field in the sentence.
    /// \param i The index of the field to set.
    /// \param value The new value to assign to the field.
    void set_field(uint32_t i, const std::string& value);
    /// \brief Gets the NMEA sentence of the sentence.
    /// \return The NMEA sentence of the sentence.
    std::string p_nmea_sentence() const;

private:
    // VARIABLES
    /// \brief Stores the sentence talker.
    std::string m_talker;
    /// \brief Stores the sentence type.
    std::string m_type;
    /// \brief Stores the fields of the sentence.
    std::vector<std::string> m_fields;

    // METHODS
    /// \brief Calculates the checksum of an NMEA sentence.
    /// \param nmea_sentence The NMEA sentence to calculate the checksum for.
    /// \return The NMEA-compliant checksum string of the NMEA sentence.
    static std::string calculate_checksum(std::string nmea_sentence);
};

}

#endif