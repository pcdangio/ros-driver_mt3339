/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>

///
/// \brief Implements the driver's ROS node functionality.
///
class ros_node
{
public:
    // CONSTRUCTORS
    ///
    /// \brief ros_node Initializes the ROS node.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ///
    ros_node(int argc, char **argv);
    ~ros_node();

    // METHODS
    ///
    /// \brief spin Runs the node.
    ///
    void spin();

private:
    // VARIABLES
    ///
    /// \brief m_driver The driver instance.
    ///
    driver* m_driver;
    ///
    /// \brief m_node The node's handle.
    ///
    ros::NodeHandle* m_node;
    ///
    /// \brief m_publisher The node's NavSatFix message publisher.
    ///
    ros::Publisher m_nav_publisher;
    ///
    /// \brief m_time_publisher The node's TimeReference message publisher.
    ///
    ros::Publisher m_time_publisher;
    ///
    /// \brief m_scan_rate The rate at which to scan for NMEA messages.
    ///
    ros::Rate* m_scan_rate;
    ///
    /// \brief m_uere Stores the User Equivalent Range Error (UERE) for the sensor.
    ///
    double m_uere;

    // METHODS
    ///
    /// \brief data_callback The callback for processing new GPS data.
    /// \param data The most recent GPS data.
    ///
    void data_callback(driver::data data);
};

#endif // ROS_NODE_H

