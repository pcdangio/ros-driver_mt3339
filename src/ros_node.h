/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>

/// \brief Implements the driver's ROS node functionality.
class ros_node
{
public:
    // CONSTRUCTORS
    /// \brief Initializes the ROS node.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ros_node(int32_t argc, char **argv);
    ~ros_node();

    // METHODS
    /// \brief Runs the node.
    void run();

private:
    // VARIABLES
    /// \brief The driver instance.
    driver* m_driver;
    /// \brief The node's handle.
    ros::NodeHandle* m_node;
    /// \brief The node's NavSatFix message publisher.
    ros::Publisher m_nav_publisher;
    /// \brief The node's TimeReference message publisher.
    ros::Publisher m_time_publisher;
    /// \brief Stores the User Equivalent Range Error (UERE) for the sensor.
    double p_uere;

    // METHODS
    /// \brief Connects the driver to the MT3339.
    /// \param port The serial port to interface with the MT3339 through.
    /// \param baud The desired baud rate to be used.
    /// \returns TRUE if the driver was able to successfully connect, otherwise FALSE.
    bool driver_connect(std::string port, uint32_t baud);
    /// \brief Configures the MT3339.
    /// \param rate The time period of NMEA udpates in milliseconds.
    /// \returns TRUE if the MT3339 was configured successfully, otherwise FALSE.
    bool driver_configure(uint32_t update_rate);

    // NMEA CALLBACKS
    /// \brief The callback for handling GGA sentences.
    void callback_gga(std::shared_ptr<nmea::gga> gga);
    /// \brief The callback for handling GSA sentences.
    void callback_gsa(std::shared_ptr<nmea::gsa> gsa);
    /// \brief The callback for handling RMC sentences.
    void callback_rmc(std::shared_ptr<nmea::rmc> rmc);
};

#endif