#include "ros_node.h"

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>

// CONSTRUCTORS
ros_node::ros_node(int argc, char **argv)
{
    // Initialize the ROS node.
    ros::init(argc, argv, "driver_mtk3339");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle();

    // Read standard parameters.
    ros::NodeHandle private_node("~");
    std::string param_serial_port;
    private_node.param<std::string>("serial_port", param_serial_port, "/dev/ttyAMA0");
    int param_baud_rate;
    private_node.param<int>("baud_rate", param_baud_rate, 115200);
    double param_scan_rate;
    private_node.param<double>("scan_rate", param_scan_rate, 15.0);
    private_node.param<double>("uere", ros_node::m_uere, 6.74);

    // Read configure parameters.
    int param_update_baud;
    private_node.param<int>("update_baud", param_update_baud, -1);
    int param_update_rate;
    private_node.param<int>("update_nmea_rate", param_update_rate, -1);

    // Set up publishers.
    ros_node::m_nav_publisher = ros_node::m_node->advertise<sensor_msgs::NavSatFix>("gps/position", 1);
    ros_node::m_time_publisher = ros_node::m_node->advertise<sensor_msgs::TimeReference>("gps/time", 1);

    // Initialize ros node members.
    ros_node::m_scan_rate = new ros::Rate(param_scan_rate);

    // Initialize driver.
    ros_node::m_driver = new driver(param_serial_port, static_cast<unsigned int>(param_baud_rate));
    ROS_INFO_STREAM("Connecting to MTK3339 on " << param_serial_port << " at " << param_baud_rate << "bps.");

    // Check if baud rate is getting updated.
    if(param_update_baud != -1)
    {
        // Update baud rate.
        try
        {
            // Send command.
            ros_node::m_driver->set_baud(static_cast<unsigned int>(param_update_baud));
            // Restart node on new baud rate.
            delete ros_node::m_driver;
            ros_node::m_driver = new driver(param_serial_port, static_cast<unsigned int>(param_update_rate));
            ROS_INFO_STREAM("Baud updated to " << param_update_baud <<"bps.");
        }
        catch (std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
        }
    }

    // Check if update rate is getting updated.
    if(param_update_rate != -1)
    {
        // Update the rate.
        try
        {
            ros_node::m_driver->set_nmea_update_rate(static_cast<unsigned int>(param_update_rate));
            ROS_INFO_STREAM("NMEA update rate set to " << param_update_rate << "ms.");
        }
        catch (std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
        }
    }

    // Attach callbacks.
    ros_node::m_driver->attach_data_callback(std::bind(&ros_node::data_callback, this, std::placeholders::_1));
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_scan_rate;
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

// METHODS
void ros_node::spin()
{
    while(ros::ok())
    {
        // Scan for NMEA strings.
        ros_node::m_driver->spin_once();

        // Spin the ros node once.
        ros::spinOnce();

        // Loop.
        ros_node::m_scan_rate->sleep();
    }
}

// CALLBACKS
void ros_node::data_callback(driver::data data)
{
    // Populate nav sat message.
    sensor_msgs::NavSatFix nav_message;
    nav_message.header.stamp = ros::Time::now();
    nav_message.header.frame_id = "driver_mtk3339";
    nav_message.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    if(data.fix == driver::data::fix_type::NONE)
    {
        // No fix.
        nav_message.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        nav_message.latitude = std::numeric_limits<double>::quiet_NaN();
        nav_message.longitude = std::numeric_limits<double>::quiet_NaN();;
        nav_message.altitude = std::numeric_limits<double>::quiet_NaN();
        nav_message.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        nav_message.position_covariance.fill(std::numeric_limits<double>::quiet_NaN());
    }
    else
    {
        // 2D or 3D fix.
        nav_message.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        nav_message.latitude = data.latitude;
        nav_message.longitude = data.longitude;
        nav_message.altitude = std::numeric_limits<double>::quiet_NaN();
        // Set covariance matrix.
        nav_message.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        // From wikipedia: https://en.wikipedia.org/wiki/Error_analysis_for_the_Global_Positioning_System
        // 3*sigma_r = UERE
        // sigma_rc = sqrt(DOP^2 * sigma_r^2  + sigma_numerical^2)
        // Calculate cov_h = HDOP^2 * (UERE/3)^2 + 1^2, cov_v = VDOP^2 * (UERE/3)^2 + 1^2
        double cov_h = std::pow(static_cast<double>(data.hdop), 2.0) * std::pow((ros_node::m_uere / 3.0), 2.0) + 1.0;
        double cov_v = std::pow(static_cast<double>(data.vdop), 2.0) * std::pow((ros_node::m_uere / 3.0), 2.0) + 1.0;
        nav_message.position_covariance = {cov_h, 0.0, 0.0,
                                           0.0, cov_h, 0.0,
                                           0.0, 0.0, cov_v};
    }
    if(data.fix == driver::data::fix_type::FIX_3D)
    {
        // 3D fix.
        nav_message.altitude = data.altitude;
    }
    // Publish time message.
    ros_node::m_nav_publisher.publish(nav_message);

    // Populate time message.
    sensor_msgs::TimeReference time_message;
    time_message.header.stamp = ros::Time::now();
    time_message.source = "MTK3339 GPS";
    time_message.time_ref = ros::Time(data.utc_time);
    // Publish time message.
    ros_node::m_time_publisher.publish(time_message);
}
