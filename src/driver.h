/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include "message.h"
#include "nmea_gga.h"
#include "nmea_gsa.h"
#include "nmea_rmc.h"

#include <serial/serial.h>

#include <boost/thread.hpp>

#include <functional>
#include <memory>
#include <mutex>

/// \brief A driver for the MT3339 GPS.
class driver
{
public:
    // ENUMERATIONS
    /// \brief An enumeration of acknowledge types.
    enum class ack_t
    {
        INVALID = 0,        ///< Invalid command/packet.
        UNSUPPORTED = 1,    ///< Unsupported command.
        FAILED = 2,         ///< Command failed.
        SUCCESS = 3,        ///< Command succeeded.
        NONE = 4            ///< No ACK was received.
    };

    // CONSTRUCTORS
    /// \brief Instantiates a new driver and opens communications with the MT3339.
    /// \param port The serial port connected to the MT3339.
    /// \param baud_rate The baud rate to communicate with the MT3339 over.
    driver(std::string port, unsigned int baud_rate);
    ~driver();

    // COMMANDS
    /// \brief Tests if the serial connection with the MT3339 is valid.
    /// \returns TRUE if the connection is valid, otherwise FALSE.
    bool test_connection();
    /// \brief Updates the baud rate that the MT3339 operates on.
    /// \param baud_rate The new baud rate.
    void set_baud(unsigned int baud_rate);
    /// \brief Sets the NMEA fix update rate.
    /// \param milliseconds The number of milliseconds between updates.
    /// \param ack OPTIONAL Passback of ACK from receiver.
    /// \return TRUE if succeeded, otherwise FALSE.
    bool set_nmea_update_rate(unsigned int milliseconds, ack_t* ack = nullptr);
    /// \brief Sets the NMEA outputs to match attached callbacks.
    /// \param ack OPTIONAL Passback of ACK from receiver.
    /// \return TRUE if succeeded, otherwise FALSE.
    bool set_nmea_output(ack_t* ack = nullptr);

    // CALLBACKS
    /// \brief Attaches a callback for handling GGA messages.
    /// \param callback The callback function to attach.
    /// \note set_nmea_output should be called after callbacks are attached.
    void attach_callback_gga(std::function<void(std::shared_ptr<nmea::gga>)> callback);
    /// \brief Attaches a callback for handling GSA messages.
    /// \param callback The callback function to attach.
    /// \note set_nmea_output should be called after callbacks are attached.
    void attach_callback_gsa(std::function<void(std::shared_ptr<nmea::gsa>)> callback);
    /// \brief Attaches a callback for handling RMC messages.
    /// \param callback The callback function to attach.
    /// \note set_nmea_output should be called after callbacks are attached.
    void attach_callback_rmc(std::function<void(std::shared_ptr<nmea::rmc>)> callback);

private:
    // VARIABLES
    /// \brief The serial port for communicating with the MT3339.
    /// \note Has internal mutexes for read and write so is thread safe.
    serial::Serial* m_port;

    /// \brief The serial port's read thread.
    boost::thread m_thread;
    std::atomic<bool> f_stop_requested;
    /// \brief The read thread worker.
    void read_thread();

    // CALLBACKS
    /// \brief Stores the callback for GGA messages.
    std::function<void(std::shared_ptr<nmea::gga>)> m_callback_gga;
    /// \brief Stores the callback for GSA messages.
    std::function<void(std::shared_ptr<nmea::gsa>)> m_callback_gsa;
    /// \brief Stores the callback for RMC messages.
    std::function<void(std::shared_ptr<nmea::rmc>)> m_callback_rmc;

    // METHODS
    /// \brief Sends a PMTK message to the MT3339.
    /// \param msg The PMTK message to send.
    void send_message(const message& msg);

    // HANDLERS
    /// \brief Handles an ACK message.
    /// \param msg The message to handle.
    void handle_ack(const message& msg);
    /// \brief Handles a TXT message.
    /// \param msg The message to handle.
    void handle_txt(const message& msg);
    /// \brief Handles a GGA message.
    /// \param msg The message to handle.
    void handle_gga(const message& msg);
    /// \brief Handles a GSA message.
    /// \param msg The message to handle.
    void handle_gsa(const message& msg);
    /// \brief Handles an RMC message.
    /// \param msg The message to handle.
    void handle_rmc(const message& msg);

    // LAST ACK
    /// \brief Retrieves an ACK message from the MT3339.
    /// \param ack The ack_t to store the retrieved ACK message in.
    /// \return TRUE if an ACK message was retrieved, otherwise FALSE.
    bool get_ack(const std::string& command, ack_t* ack);
    /// \brief Records the last ACK message received from the MT3339.
    struct last_ack_t
    {
        /// \brief Indicates if this instance is set/new.
        bool is_set;
        /// \brief The command that the ack is associated with.
        std::string command;
        /// \brief The value of the acknowledge.
        ack_t ack;
    };
    /// \brief Stores the last received ACK message.
    last_ack_t m_last_ack;
    /// \brief Thread safety for m_last_ack.
    std::mutex m_mutex_ack;

    // LAST TXT
    /// \brief Retrieves a TXT message from the MT3339.
    /// \param text The string to store the retrieved text in.
    /// \returns TRUE if a TXT message was retrieved, otherwise FALSE.
    bool get_txt(std::string& text);
    /// \brief Records the last TXT messsage received from the MT3339.
    struct last_txt_t
    {
        bool is_set;
        std::string text;
    };
    /// \brief Stores the last received TXT message.
    last_txt_t m_last_txt;
    /// \brief Threat safety for m_last_txt.
    std::mutex m_mutex_txt;
};

#endif // DRIVER_H
