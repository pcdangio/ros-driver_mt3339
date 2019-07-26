/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include <serial/serial.h>
#include "message.h"

#include <map>
#include <functional>

///
/// \brief A driver for the MTK3339 GPS.
///
class driver
{
public:
    // STRUCTURES
    ///
    /// \brief A data structure for storing/sharing received GPS data.
    ///
    struct data
    {
        ///
        /// \brief An enumeration of GPS fix types.
        ///
        enum class fix_type
        {
            NONE = 1,       ///< No fix.
            FIX_2D = 2,     ///< 2D fix (<4 Satellites)
            FIX_3D = 3      ///< 3D fix (>= 4 Satellites)
        };

        ///
        /// \brief utc_time GPS provided UTC time, in seconds.
        ///
        double utc_time;
        ///
        /// \brief fix The current fix status.
        ///
        fix_type fix;

        ///
        /// \brief latitude The latitude in degrees, positive North.
        ///
        double latitude;
        ///
        /// \brief longitude The longitude in degrees, positive East.
        ///
        double longitude;
        ///
        /// \brief altitude The MSL altitude in meters.
        ///
        double altitude;
        ///
        /// \brief hdop The horizontal dilution of precision.
        ///
        double hdop;
        ///
        /// \brief vdop The vertical dilution of precision.
        ///
        double vdop;
    };

    // CONSTRUCTORS
    ///
    /// \brief driver Instantiates a new driver and opens communications with the MTK3339.
    /// \param port The serial port connected to the MTK3339.
    /// \param baud_rate The baud rate to communicate with the MTK3339 over.
    ///
    driver(std::string port, unsigned int baud_rate);
    ~driver();

    // METHODS
    ///
    /// \brief set_baud Command for updating the baud rate that the MTK3339 operates on.
    /// \param baud_rate The new baud rate.
    ///
    void set_baud(unsigned int baud_rate);
    ///
    /// \brief set_nmea_update_rate Command for updating the NMEA fix update rate.
    /// \param milliseconds The number of milliseconds between updates.
    ///
    void set_nmea_update_rate(unsigned int milliseconds);
    ///
    /// \brief set_nmea_output Command for setting the NMEA outputs to GGA, GSA, RMC.
    ///
    void set_nmea_output();
    ///
    /// \brief attach_data_callback Attaches a callback for handling new GPS data.
    /// \param callback The callback function to call when new GPS data is available.
    ///
    void attach_data_callback(std::function<void(data)> callback);
    ///
    /// \brief spin_once Spins a single iteration of the driver, which reads NMEA messages from the MTK3339.
    ///
    void spin_once();

private:
    // VARIABLES
    ///
    /// \brief m_port The serial port for communicating with the MTK3339.
    ///
    serial::Serial* m_port;
    ///
    /// \brief m_current_data A storage structure for building recent data as it is received.
    ///
    data m_current_data;
    ///
    /// \brief m_data_ready A 3-bit field indicating if GGA, GSA, and RMC data has been received.
    ///
    int m_data_ready;
    ///
    /// \brief m_callback The callback function to raise when data is ready.
    ///
    std::function<void(data)> m_callback;

    // METHODS
    ///
    /// \brief send_message Sends a PMTK message to the GPS.
    /// \param msg The PMTK message to send.
    ///
    void send_message(message msg);
    ///
    /// \brief read_message Reads the next NMEA sentence from the serial buffer.
    /// \return Returns a valid pointer to a read message if available. Returns NULL if the read timed out.
    ///
    message* read_message();
    ///
    /// \brief handle_gp Handles NMEA messages from GP talkers, populating m_current_data and raising m_callback after m_data_ready.
    /// \param msg The GP message to parse.
    ///
    void handle_gp(const message* msg);
};

#endif // DRIVER_H
