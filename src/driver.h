/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include "nmea_sentence.h"

#include <ros/ros.h>
#include <serial/serial.h>

#include <sensor_msgs_ext/gnss_fix.h>
#include <sensor_msgs_ext/gnss_position.h>
#include <sensor_msgs_ext/gnss_track.h>
#include <sensor_msgs_ext/time_reference.h>

#include <boost/thread.hpp>

/// \brief A driver for the MT3339 GPS.
class driver
{
public:
    // CONSTRUCTORS
    /// \brief Initializes the ROS node.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    driver(int32_t argc, char **argv);
    ~driver();

    // METHODS
    /// \brief Runs the node.
    void run();    

private:
    // PARAMETERS
    /// \brief Stores the frame ID of the receiver's coordinate frame.
    std::string p_frame_id;
    /// \brief Stores the User Equivalent Range Error (UERE) for the sensor.
    double p_uere;

    // ROS NODE
    /// \brief The node's handle.
    ros::NodeHandle* m_node;

    // ROS PUBLISHERS
    /// \brief Publisher for GPS fix information.
    ros::Publisher m_publisher_gnss_fix;
    /// \brief Publisher for GPS position information.
    ros::Publisher m_publisher_gnss_position;
    /// \brief Publisher for GPS track information.
    ros::Publisher m_publisher_gnss_track;
    /// \brief Publisher for GPS time information.
    ros::Publisher m_publisher_time_reference;

    // MESSAGE BUILDERS
    /// \brief A message builder for GNSS fix messages.
    sensor_msgs_ext::gnss_fix* m_builder_gnss_fix;
    /// \brief A message builder for GNSS position messages.
    sensor_msgs_ext::gnss_position* m_builder_gnss_position;

    // COMPONENTS
    /// \brief The serial port for communicating with the MT3339.
    /// \note Has internal mutexes for read and write so is thread safe.
    serial::Serial* m_port;

    // THREADING
    /// \brief The serial port's read thread.
    boost::thread m_thread;
    /// \brief Indicates if the read thread is running.
    std::atomic<bool> f_is_reading;
    /// \brief Indicates if a stop has been requested for the read thread.
    std::atomic<bool> f_stop_requested;
    /// \brief The read thread worker.
    void read_thread();

    // CONNECTION
    /// \brief Makes a validated connection with the MT3339 at a desired baud rate.
    /// \param port The serial port connected to the MT3339.
    /// \param desired_baud The desired baud rate to communicate with the MT3339 over.
    /// \returns TRUE if the connection was made, otherwise FALSE.
    /// \details This will find the correct baud rate and transition to the desired baud rate if necessary.
    bool make_connection(std::string port, uint32_t desired_baud);
    /// \brief Opens a connection with the MT3339 as a specific baud rate.
    /// \param port The serial port connected to the MT3339.
    /// \param baud The baud rate to communicate with the MT3339 over.
    /// \returns TRUE if the serial port was opened successfully, otherwise FALSE.
    bool connect(std::string port, uint32_t baud);
    /// \brief Closes the connection with the MT3339.
    void disconnect();
    /// \brief Tests for a functional connection with the MT3339.
    /// \return TRUE if the connection is validated, otherwise FALSE.
    bool test_connection();

    // IO
    /// \brief Sends a PMTK sentence to the MT3339.
    /// \param sentence The PMTK sentence to send.
    void send_sentence(const nmea::sentence& sentence);

    // COMMANDS
    /// \brief Updates the baud rate that the MT3339 operates on.
    /// \param baud_rate The new baud rate.
    void set_baud(uint32_t baud_rate);
    /// \brief Sets the NMEA outputs to match attached callbacks.
    /// \return TRUE if succeeded, otherwise FALSE.
    bool set_nmea_output(); 
    /// \brief Sets the NMEA fix update rate.
    /// \param milliseconds The number of milliseconds between updates.
    /// \return TRUE if succeeded, otherwise FALSE.
    bool set_nmea_update_rate(uint32_t milliseconds);

    // HANDLERS
    /// \brief Handles an ACK sentence.
    /// \param sentence The sentence to handle.
    void handle_ack(const nmea::sentence& sentence);
    /// \brief Handles a 705 sentence.
    /// \param sentence The sentence to handle.
    void handle_705(const nmea::sentence& sentence);
    /// \brief Handles a GGA sentence.
    /// \param sentence The sentence to handle.
    void handle_gga(const nmea::sentence& sentence);
    /// \brief Handles a GSA sentence.
    /// \param sentence The sentence to handle.
    void handle_gsa(const nmea::sentence& sentence);
    /// \brief Handles an RMC sentence.
    /// \param sentence The sentence to handle.
    void handle_rmc(const nmea::sentence& sentence);

    // FLAGS
    /// \brief Indicates if the MT3339 responded to a FW version request.
    std::atomic<bool> f_connection_ok;
    /// \brief Stores if a sucessfull ACK was received by the MT3339.
    std::atomic<bool> f_received_ack;
};

#endif // DRIVER_H
