/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>

#include <sensor_msgs_ext/gnss_fix.h>
#include <sensor_msgs_ext/gnss_position.h>
#include <sensor_msgs_ext/gnss_track.h>
#include <sensor_msgs_ext/time_reference.h>

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
    // COMPONENTS
    /// \brief The driver instance.
    driver* m_driver;

    // NODE
    /// \brief The node's handle.
    ros::NodeHandle* m_node;

    // PARAMETERS
    /// \brief Stores the frame ID of the receiver's coordinate frame.
    std::string p_frame_id;
    /// \brief Stores the User Equivalent Range Error (UERE) for the sensor.
    double p_uere;

    // PUBLISHERS
    /// \brief Publisher for GPS fix information.
    ros::Publisher m_publisher_gnss_fix;
    /// \brief Publisher for GPS position information.
    ros::Publisher m_publisher_gnss_position;
    /// \brief Publisher for GPS track information.
    ros::Publisher m_publisher_gnss_track;
    /// \brief Publisher for GPS time information.
    ros::Publisher m_publisher_time_reference;

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

    // MESSAGING
    uint8_t m_group_tracker;
    bool f_has_fix;
    sensor_msgs_ext::gnss_fix* m_gnss_fix;
    sensor_msgs_ext::gnss_position* m_gnss_position;
    sensor_msgs_ext::gnss_track* m_gnss_track;
    sensor_msgs_ext::time_reference* m_time_reference;
    void publish_messages();
};

#endif