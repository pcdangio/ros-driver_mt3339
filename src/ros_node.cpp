#include "ros_node.h"

#include <deque>

// CONSTRUCTORS
ros_node::ros_node(int argc, char **argv)
{
    // Initialize the ROS node.
    ros::init(argc, argv, "driver_mt3339");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle("~");
    ros::NodeHandle public_node;

    // Read standard parameters.
    std::string p_port = ros_node::m_node->param<std::string>("serial_port", "/dev/ttyAMA0");
    uint32_t p_baud = ros_node::m_node->param<int32_t>("baud_rate", 38400);
    uint32_t p_update_rate = ros_node::m_node->param<int32_t>("update_rate", 100);
    ros_node::p_frame_id = ros_node::m_node->param<std::string>("frame_id", "mt3339");
    ros_node::p_uere = ros_node::m_node->param<double>("uere", 6.74);

    // Initialize builders.
    ros_node::m_group_tracker = 0;

    // Set up publishers.
    ros_node::m_publisher_gnss_fix = public_node.advertise("gnss/fix", 1);
    ros_node::m_publisher_gnss_position = public_node.advertise("gnss/position", 1);
    ros_node::m_publisher_gnss_track = public_node.advertise("gnss/track", 1);
    ros_node::m_publisher_time_reference = public_node.advertise("gnss/time", 1);

    // Connect the driver to the MT3339.
    if(!ros_node::driver_connect(p_port, p_baud))
    {
        ROS_FATAL("unable to connect to MT3339");
        exit(1);
    }

    // Initialize MT3339.
    if(!ros_node::driver_configure(p_update_rate))
    {
        ROS_FATAL("unable to configure MT3339");
        exit(2);
    }
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

// METHODS
bool ros_node::driver_connect(std::string port, uint32_t baud)
{
    // Build a list of baud rates to cycle through (in order of likelihood)
    std::deque<uint32_t> baud_rates = {9600,19200,38400,57600,115200,4800};
    // Remove the desired baud rate from the list.
    auto to_remove = std::find(baud_rates.begin(), baud_rates.end(), baud);
    if(to_remove != baud_rates.end())
    {
        baud_rates.erase(to_remove);
    }
    // Insert desired baud at beginning of attempt list.
    baud_rates.push_front(baud);

    // Iterate through baud rates and attempt to connect.
    for(auto baud_rate = baud_rates.cbegin(); baud_rate != baud_rates.cend(); ++baud_rate)
    {
        ROS_INFO_STREAM("attempting to connect to MT3339 on " << port << ":" << *baud_rate << "baud...");;
        // Open the port with the current baud rate setting.
        try
        {
            ros_node::m_driver = new driver(port, *baud_rate);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("error opening port " << port << " (" << e.what() << ")");
            return false;
        }
        // Give serial connection time to settle before testing connection.
        usleep(250000);
        // Test the connection.
        if(ros_node::m_driver->test_connection())
        {
            // Connection worked.
            ROS_INFO("successfully connected to MT3339");
            // Check if this is the desired baud rate.
            if(*baud_rate != baud)
            {
                ROS_INFO_STREAM("switching baud rate to " << baud << "bps...");
                // Change the baud rate.
                ros_node::m_driver->set_baud(baud);
                // Close the port.
                delete ros_node::m_driver;
                // Recurse to re-open the port and test its connection.
                return ros_node::driver_connect(port, baud);
            }
            else
            {
                return true;
            }
        }
        else
        {
            // Connection failed.
            ROS_WARN("connection attempt failed.");
            // Close port.
            delete ros_node::m_driver;
            ros_node::m_driver = nullptr;
        }

        if(!ros::ok())
        {
            break;
        }
    }

    // If this point reached, all connection attempts have failed.
    return false;
}
bool ros_node::driver_configure(uint32_t update_rate)
{
    // Attach callbacks before initializing driver.
    ros_node::m_driver->attach_callback_gga(std::bind(&ros_node::callback_gga, this, std::placeholders::_1));
    ros_node::m_driver->attach_callback_gsa(std::bind(&ros_node::callback_gsa, this, std::placeholders::_1));
    ros_node::m_driver->attach_callback_rmc(std::bind(&ros_node::callback_rmc, this, std::placeholders::_1));

    // Set NMEA output to match attached callbacks.
    if(!ros_node::m_driver->set_nmea_output())
    {
        ROS_ERROR("failed to configure nmea sentence output");
        return false;
    }

    // Set NMEA update rate.
    if(!ros_node::m_driver->set_nmea_update_rate(update_rate))
    {
        ROS_ERROR("failed to configure nmea update rate");
        return false;
    }

    return true;
}
void ros_node::run()
{
    // Spin ros to keep node alive while serial port thread reads in background.
    ros::spin();
}

// CALLBACKS
void ros_node::callback_gga(std::shared_ptr<nmea::gga> gga)
{
    // Clear fix and position messages.
    delete ros_node::m_gnss_fix;
    delete ros_node::m_gnss_position;
    ros_node::m_gnss_fix = new sensor_msgs_ext::gnss_fix();
    ros_node::m_gnss_position = new sensor_msgs_ext::gnss_position();

    // Populate fix message components.
    ros_node::m_gnss_fix->type = static_cast<uint8_t>(gga->fix_type);
    ros_node::m_gnss_fix->satellite_count = gga->satellite_count;

    // Store if the receiver currently has a fix.
    ros_node::f_has_fix = gga->fix_type > nmea::gga::fix_type_t::NONE;

    // Populate position message components.
    if(ros_node::f_has_fix)
    {
        ros_node::m_gnss_position->frame_id = ros_node::p_frame_id;
        ros_node::m_gnss_position->latitude = gga->latitude;
        ros_node::m_gnss_position->longitude = gga->longitude;
        ros_node::m_gnss_position->altitude = gga->altitude;
    }

    // GGA should be first message received in group. Reset nmea group tracker.
    ros_node::m_group_tracker = 1; // 0b0001
}
void ros_node::callback_gsa(std::shared_ptr<nmea::gsa> gsa)
{
    // Populate fix message components.
    ros_node::m_gnss_fix->mode_selection = static_cast<uint8_t>(gsa->mode_selection);
    ros_node::m_gnss_fix->mode = static_cast<uint8_t>(gsa->mode);

    // Populate position message components.
    ros_node::m_gnss_position->fix_3d = gsa->mode == nmea::gsa::mode_t::_3D;
    ros_node::m_gnss_position->has_covariance = true;
    // From wikipedia: https://en.wikipedia.org/wiki/Error_analysis_for_the_Global_Positioning_System
    // 3*sigma_r = UERE
    // sigma_rc = sqrt(DOP^2 * sigma_r^2  + sigma_numerical^2)
    // Calculate cov_h = HDOP^2 * (UERE/3)^2 + 1^2, cov_v = VDOP^2 * (UERE/3)^2 + 1^2
    double cov_h = std::pow(static_cast<double>(gsa->hdop), 2.0) * std::pow((ros_node::p_uere / 3.0), 2.0) + 1.0;
    double cov_v = std::pow(static_cast<double>(gsa->vdop), 2.0) * std::pow((ros_node::p_uere / 3.0), 2.0) + 1.0;
    ros_node::m_gnss_position->covariance = {cov_h, 0.0, 0.0,
                                             0.0, cov_h, 0.0,
                                             0.0, 0.0, cov_v};

    // Append GSA to nmea group tracker.
    ros_node::m_group_tracker |= 2; // 0b0010
}
void ros_node::callback_rmc(std::shared_ptr<nmea::rmc> rmc)
{
    

    // Append RMC to nmea group tracker.
    ros_node::m_group_tracker |= 4; // 0b0100

    // RMC is last expected message. Run publisher.
    ros_node::publish_messages();
}
// void ros_node::data_callback(driver::data data)
// {
//     // Populate nav sat message.
//     sensor_msgs::NavSatFix nav_message;
//     nav_message.header.stamp = ros::Time::now();
//     nav_message.header.frame_id = "driver_mtk3339";
//     nav_message.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
//     if(data.fix == driver::data::fix_type::NONE)
//     {
//         // No fix.
//         nav_message.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
//         nav_message.latitude = std::numeric_limits<double>::quiet_NaN();
//         nav_message.longitude = std::numeric_limits<double>::quiet_NaN();;
//         nav_message.altitude = std::numeric_limits<double>::quiet_NaN();
//         nav_message.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
//         nav_message.position_covariance.fill(std::numeric_limits<double>::quiet_NaN());
//     }
//     else
//     {
//         // 2D or 3D fix.
//         nav_message.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
//         nav_message.latitude = data.latitude;
//         nav_message.longitude = data.longitude;
//         nav_message.altitude = std::numeric_limits<double>::quiet_NaN();
//         // Set covariance matrix.
//         nav_message.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
//         // From wikipedia: https://en.wikipedia.org/wiki/Error_analysis_for_the_Global_Positioning_System
//         // 3*sigma_r = UERE
//         // sigma_rc = sqrt(DOP^2 * sigma_r^2  + sigma_numerical^2)
//         // Calculate cov_h = HDOP^2 * (UERE/3)^2 + 1^2, cov_v = VDOP^2 * (UERE/3)^2 + 1^2
//         double cov_h = std::pow(static_cast<double>(data.hdop), 2.0) * std::pow((ros_node::m_uere / 3.0), 2.0) + 1.0;
//         double cov_v = std::pow(static_cast<double>(data.vdop), 2.0) * std::pow((ros_node::m_uere / 3.0), 2.0) + 1.0;
//         nav_message.position_covariance = {cov_h, 0.0, 0.0,
//                                            0.0, cov_h, 0.0,
//                                            0.0, 0.0, cov_v};
//     }
//     if(data.fix == driver::data::fix_type::FIX_3D)
//     {
//         // 3D fix.
//         nav_message.altitude = data.altitude;
//     }
//     // Publish time message.
//     ros_node::m_nav_publisher.publish(nav_message);

//     // Populate time message.
//     sensor_msgs::TimeReference time_message;
//     time_message.header.stamp = ros::Time::now();
//     time_message.source = "MTK3339 GPS";
//     time_message.time_ref = ros::Time(data.utc_time);
//     // Publish time message.
//     ros_node::m_time_publisher.publish(time_message);
// }
