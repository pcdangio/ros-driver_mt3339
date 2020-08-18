#include "driver.h"

#include <chrono>

// CONSTRUCTORS
driver::driver(int32_t argc, char **argv)
{
    // Initialize the ROS node.
    ros::init(argc, argv, "driver_mt3339");

    // Get the node's handle.
    driver::m_node = new ros::NodeHandle("~");    

    // Read parameters.
    driver::p_connection_settle_time = driver::m_node->param<int32_t>("connection_settle_time", 300);
    driver::p_timeout = driver::m_node->param<int32_t>("timeout", 300);
    std::string p_port = driver::m_node->param<std::string>("serial_port", "/dev/ttyAMA0");
    uint32_t p_baud = driver::m_node->param<int32_t>("baud_rate", 38400);
    uint32_t p_update_rate = driver::m_node->param<int32_t>("update_rate", 100);
    driver::p_frame_id = driver::m_node->param<std::string>("frame_id", "mt3339");
    driver::p_uere = driver::m_node->param<double>("uere", 6.74);

    // Set up publishers.
    ros::NodeHandle public_node;
    driver::m_publisher_gnss_fix = public_node.advertise<sensor_msgs_ext::gnss_fix>("gnss/fix", 1);
    driver::m_publisher_gnss_position = public_node.advertise<sensor_msgs_ext::gnss_position>("gnss/position", 1);
    driver::m_publisher_gnss_track = public_node.advertise<sensor_msgs_ext::gnss_track>("gnss/track", 1);
    driver::m_publisher_time_reference = public_node.advertise<sensor_msgs_ext::time_reference>("gnss/time", 1);
    
    // Initialize flags.
    driver::f_is_reading = false;
    driver::f_stop_requested = false;
    driver::f_connection_ok = false;
    driver::f_received_ack = false;

    // Initialize builders.
    driver::m_builder_gnss_fix = nullptr;
    driver::m_builder_gnss_position = nullptr;

    // Make the connection with the MT3339.
    if(!driver::make_connection(p_port, p_baud))
    {
        ROS_FATAL("unable to connect to MT3339");
        exit(1);
    }

    // Configure NMEA output.
    if(!driver::set_nmea_output())
    {
        ROS_FATAL("unable to configure NMEA outputs");
        exit(1);
    }
    
    // Configure NMEA generation rate.
    if(!driver::set_nmea_update_rate(p_update_rate))
    {
        ROS_FATAL("unable to configure NMEA update rate");
        exit(1);
    }
}
driver::~driver()
{
    // Disconnect from the serial port.
    driver::disconnect();

    // Close down the node.
    delete driver::m_node;

    // Clean up builders.
    delete driver::m_builder_gnss_fix;
    delete driver::m_builder_gnss_position;
}

// RUN
void driver::run()
{
    ros::spin();
}

// CONNECTION
bool driver::make_connection(std::string port, uint32_t desired_baud)
{
    // Build a list of baud rates to cycle through (in order of likelihood)
    std::deque<uint32_t> baud_rates = {9600,19200,38400,57600,115200,4800};
    // Remove the desired baud rate from the list.
    auto to_remove = std::find(baud_rates.begin(), baud_rates.end(), desired_baud);
    if(to_remove != baud_rates.end())
    {
        baud_rates.erase(to_remove);
    }
    // Insert desired baud at beginning of attempt list.
    baud_rates.push_front(desired_baud);

    // Iterate through baud rates and attempt to connect.
    for(auto baud_rate = baud_rates.cbegin(); baud_rate != baud_rates.cend(); ++baud_rate)
    {
        ROS_INFO_STREAM("attempting to connect to MT3339 on " << port << ":" << *baud_rate << "baud...");;
        // Open the port with the current baud rate setting.
        if(!driver::connect(port, *baud_rate))
        {
            return false;
        }
        // Give serial connection time to settle before testing connection.
        usleep(driver::p_connection_settle_time * 1000);
        // Test the connection.
        if(driver::test_connection())
        {
            // Connection worked.
            ROS_INFO("successfully connected to MT3339");
            // Check if this is the desired baud rate.
            if(*baud_rate != desired_baud)
            {
                ROS_INFO_STREAM("switching baud rate to " << desired_baud << "bps...");
                // Change the baud rate.
                driver::set_baud(desired_baud);
                // Disconnect.
                driver::disconnect();
                // Recurse to re-open the port and test its connection.
                return driver::make_connection(port, desired_baud);
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
            // Disconnect.
            driver::disconnect();
        }

        // Break if ros is shut down.
        if(!ros::ok())
        {
            break;
        }
    }

    // If this point reached, all connection attempts have failed.
    return false;
}
bool driver::connect(std::string port, uint32_t baud_rate)
{
    // Open the serial port.
    try
    {
        // Use interbyte timeout since partial NMEA strings can be sent.
        driver::m_port = new serial::Serial(port, baud_rate, serial::Timeout(10, 100, 0, 100, 0));
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("failed to open serial port at " << port << ":" << baud_rate << "bps (" << e.what() << ")");
        return false;
    }

    // Start the read thread.
    driver::f_stop_requested = false;
    driver::m_thread = boost::thread(&driver::read_thread, this);

    // Wait for thread to start.
    while(!driver::f_is_reading)
    {
        usleep(100);
    }

    return true;
}
void driver::disconnect()
{
    // Stop the read thread.
    driver::f_stop_requested = true;
    driver::m_thread.join();

    // Close the serial port.
    driver::m_port->close();
    delete driver::m_port;
    driver::m_port = nullptr;
}
bool driver::test_connection()
{
    // Clear test flag.
    driver::f_connection_ok = false;

    // Create a FW version query sentence.
    nmea::sentence sentence("PMTK", "605");

    // Send the sentence.
    driver::send_sentence(sentence);

    // Wait for response from MT3339.
    std::chrono::duration<int32_t, std::milli> timeout(driver::p_timeout);
    auto start_time = std::chrono::steady_clock::now();
    while((std::chrono::steady_clock::now() - start_time) <= timeout)
    {
        if(driver::f_connection_ok)
        {
            return true;
        }
        else
        {
            usleep(10000);
        }   
    }

    // If this point reached, the startup SYS sentence was never receieved.
    return false;
}

// IO
void driver::send_sentence(const nmea::sentence& sentence)
{
    // Write the sentence to the MT3339
    driver::m_port->write(sentence.nmea_sentence());
}
void driver::read_thread()
{
    driver::m_port->flushInput();

    // Set running flag.
    driver::f_is_reading = true;

    // Loop until stopped.
    while(!driver::f_stop_requested)
    {
        // Read the next line NMEA sentence.
        std::string buffer;
        unsigned long bytes_read = driver::m_port->readline(buffer, 200, "\r\n");
        // Check if anything was read.
        if(bytes_read == 0)
        {
            continue;
        }

        // Validate the buffer as a message.
        if(!nmea::sentence::validate_checksum(buffer))
        {
            continue;
        }

        // Convert string into sentence.
        nmea::sentence sentence(buffer);

        // Handle the sentence based on talker and type.
        std::string talker = sentence.talker();
        std::string type = sentence.type();
        if(talker == "PMTK")
        {
            if(type == "001")
            {
                driver::handle_ack(sentence);
            }
            else if(type == "705")
            {
                driver::handle_705(sentence);
            }
        }
        else if(talker == "GP")
        {
            if(type == "GGA")
            {
                driver::handle_gga(sentence);
            }
            else if(type == "GSA")
            {
                driver::handle_gsa(sentence);
            }
            else if(type == "RMC")
            {
                driver::handle_rmc(sentence);
            }
        }
    }

    // Indicate that thread is no longer running.
    driver::f_is_reading = false;
}

// COMMANDS
void driver::set_baud(uint32_t baud_rate)
{
    // Build sentence to send.
    nmea::sentence sentence("PMTK", "251", 1);
    sentence.set_field(0, std::to_string(baud_rate));

    // Send sentence.
    driver::send_sentence(sentence);

    // Do not try and receive an ACK as the baud rate is changed.
}
bool driver::set_nmea_output()
{
    // Build sentence according to what callbacks are set.
    nmea::sentence sentence("PMTK", "314", 19);
    // Initialize all fields to zero.
    for(uint32_t i = 0; i < 19; ++i)
    {
        sentence.set_field(i, "0");
    }
    // Enable RMC
    sentence.set_field(1, "1");
    // ENABLE GGA
    sentence.set_field(3, "1");
    // ENABLE GSA
    sentence.set_field(4, "1");

    // Clear ack flag.
    driver::f_received_ack = false;

    // Send sentence.
    driver::send_sentence(sentence);
    
    // Retrieve the ack.
    std::chrono::duration<int32_t, std::milli> timeout(driver::p_timeout);
    auto start_time = std::chrono::steady_clock::now();
    while((std::chrono::steady_clock::now() - start_time) <= timeout)
    {
        if(driver::f_received_ack)
        {
            return true;
        }
        else
        {
            usleep(10000);
        }   
    }

    // If this point reached, the ACK was never received.
    return false;
}
bool driver::set_nmea_update_rate(uint32_t milliseconds)
{
    // Truncate the rate to the allowable range.
    milliseconds = std::min(10000U, std::max(0U, milliseconds));
    
    // Build the sentence.
    nmea::sentence sentence("PMTK", "220", 1);
    sentence.set_field(0, std::to_string(milliseconds));

    // Clear ack flag.
    driver::f_received_ack = false;

    // Send the sentence.
    driver::send_sentence(sentence);

    // Retrieve the ack.
    std::chrono::duration<int32_t, std::milli> timeout(driver::p_timeout);
    auto start_time = std::chrono::steady_clock::now();
    while((std::chrono::steady_clock::now() - start_time) <= timeout)
    {
        if(driver::f_received_ack)
        {
            return true;
        }
        else
        {
            usleep(10000);
        }   
    }

    // If this point reached, the ACK was never received.
    return false;
}

// HANDLERS
void driver::handle_ack(const nmea::sentence& sentence)
{
    driver::f_received_ack = (sentence.get_field(1) == "3");
}
void driver::handle_705(const nmea::sentence& sentence)
{
    // This message is used to test connectivity with the MT3339.
    driver::f_connection_ok = true;
}
void driver::handle_gga(const nmea::sentence& sentence)
{
    // Clear the fix and position builders.
    if(driver::m_builder_gnss_fix)
    {
        delete driver::m_builder_gnss_fix;
    }
    if(driver::m_builder_gnss_position)
    {
        delete driver::m_builder_gnss_position;
    }

    // Create a new builder for the fix message.
    driver::m_builder_gnss_fix = new sensor_msgs_ext::gnss_fix();
    // Populate the relevant portions of the fix message.
    // Parse "fix quality" as fix type.
    driver::m_builder_gnss_fix->type = std::stoi(sentence.get_field(5));
    // Parse satellite count.
    if(sentence.has_field(6))
    {
        driver::m_builder_gnss_fix->satellite_count = std::stoul(sentence.get_field(6));
    }

    // Check if a fix is available.
    if(driver::m_builder_gnss_fix->type == sensor_msgs_ext::gnss_fix::TYPE_NO_FIX)
    {
        // Quit before doing any building of the position message.
        return;
    }

    // Create a new builder for the position message.
    driver::m_builder_gnss_position = new sensor_msgs_ext::gnss_position();
    // Populate the relevant portions of the position message.
    // Parse latitude.
    if(sentence.has_field(1))
    {
        driver::m_builder_gnss_position->latitude = std::stod(sentence.get_field(1).substr(0,2));
        driver::m_builder_gnss_position->latitude += std::stod(sentence.get_field(1).substr(2)) / 60.0;
        if(sentence.get_field(2) == "S")
        {
            driver::m_builder_gnss_position->latitude *= -1;
        }
    }
    else
    {
        driver::m_builder_gnss_position->latitude = std::numeric_limits<double>::quiet_NaN();
    }
    // Parse longitude.
    if(sentence.has_field(3))
    {
        driver::m_builder_gnss_position->longitude = std::stod(sentence.get_field(3).substr(0,2));
        driver::m_builder_gnss_position->longitude += std::stod(sentence.get_field(3).substr(2)) / 60.0;
        if(sentence.get_field(4) == "W")
        {
            driver::m_builder_gnss_position->longitude *= -1;
        }
    }
    else
    {
        driver::m_builder_gnss_position->longitude = std::numeric_limits<double>::quiet_NaN();
    }
    // Parse Altitude
    if(sentence.has_field(8))
    {
        driver::m_builder_gnss_position->altitude = std::stod(sentence.get_field(8));
    }
    else
    {
        driver::m_builder_gnss_position->altitude = std::numeric_limits<double>::quiet_NaN();
    }
}
void driver::handle_gsa(const nmea::sentence& sentence)
{
    // Pull the fix mode as it will be used for both messages.
    uint8_t mode = std::stoi(sentence.get_field(1))-1;

    // Check if fix builder is set by a previous gga message.
    if(driver::m_builder_gnss_fix)
    {
        // Parse mode.
        std::string mode_selection = sentence.get_field(0);
        if(mode_selection == "A")
        {
            driver::m_builder_gnss_fix->mode_selection = sensor_msgs_ext::gnss_fix::MODE_SELECTION_AUTOMATIC;
        }
        else if(mode_selection == "M")
        {
            driver::m_builder_gnss_fix->mode_selection = sensor_msgs_ext::gnss_fix::MODE_SELECTION_MANUAL;
        }
        // Parse fix type.
        driver::m_builder_gnss_fix->mode = mode;

        // Message is now built. Send message.
        driver::m_publisher_gnss_fix.publish(*driver::m_builder_gnss_fix);
        delete driver::m_builder_gnss_fix;
        driver::m_builder_gnss_fix = nullptr;
    }

    // Check if position builder is set by a previous gga message.
    if(driver::m_builder_gnss_position)
    {
        // Set fix_3d boolean field.
        driver::m_builder_gnss_position->fix_3d = mode == sensor_msgs_ext::gnss_fix::MODE_3D;

        // Parse DOPs
        double hdop = std::numeric_limits<double>::quiet_NaN();
        double vdop = std::numeric_limits<double>::quiet_NaN();
        // Parse HDOP
        if(sentence.has_field(15))
        {
            hdop = std::stod(sentence.get_field(15));
            // Set has_covariance to true.
            driver::m_builder_gnss_position->has_covariance = true;
        }
        // Parse VDOP
        if(sentence.has_field(16))
        {
            vdop = std::stod(sentence.get_field(16));
            // Set has_covariance to true.
            driver::m_builder_gnss_position->has_covariance = true;
        }

        // From wikipedia: https://en.wikipedia.org/wiki/Error_analysis_for_the_Global_Positioning_System
        // 3*sigma_r = UERE
        // sigma_rc = sqrt(DOP^2 * sigma_r^2  + sigma_numerical^2)
        // Calculate cov_h = HDOP^2 * (UERE/3)^2 + 1^2, cov_v = VDOP^2 * (UERE/3)^2 + 1^2
        double cov_h = std::pow(static_cast<double>(hdop), 2.0) * std::pow((driver::p_uere / 3.0), 2.0) + 1.0;
        double cov_v = std::pow(static_cast<double>(vdop), 2.0) * std::pow((driver::p_uere / 3.0), 2.0) + 1.0;
        driver::m_builder_gnss_position->covariance = {cov_h, 0.0, 0.0,
                                                       0.0, cov_h, 0.0,
                                                       0.0, 0.0, cov_v};

        // Message is now built. Send message.
        driver::m_publisher_gnss_position.publish(*driver::m_builder_gnss_position);
        delete driver::m_builder_gnss_position;
        driver::m_builder_gnss_position = nullptr;
    }
}
void driver::handle_rmc(const nmea::sentence& sentence)
{
    // Check for a fix before using the RMC message for anything.
    if(sentence.get_field(1) == "V")
    {
        return;
    }

    // Create time reference message.
    sensor_msgs_ext::time_reference message_time_reference;
    // Populate time reference message.
    // Time Field (idx 0): hhmmss.sss
    // Date Field (idx 8): ddmmyy
    std::tm time_struct;
    // Convert date + hours + minutes to seconds.
    std::string field_time = sentence.get_field(0);
    std::string field_date = sentence.get_field(8);
    time_struct.tm_hour = std::stoi(field_time.substr(0, 2));
    time_struct.tm_min = std::stoi(field_time.substr(2, 2));
    time_struct.tm_mday = std::stoi(field_date.substr(0,2));
    time_struct.tm_mon = std::stoi(field_date.substr(2,2)) - 1;
    time_struct.tm_year = std::stoi(field_date.substr(4, 2)) + 100;
    double utc_seconds = static_cast<double>(timegm(&time_struct));
    // Parse and add in seconds.
    utc_seconds += std::stod(field_time.substr(4));
    message_time_reference.utc_time = ros::Time(utc_seconds);
    // Publish message.
    driver::m_publisher_time_reference.publish(message_time_reference);

    // Create track message.
    sensor_msgs_ext::gnss_track message_gnss_track;
    // Populate message.
    message_gnss_track.reference = sensor_msgs_ext::gnss_track::REFERENCE_TRUE_NORTH;
    // Parse ground speed.
    if(sentence.has_field(6))
    {
        message_gnss_track.velocity = std::stod(sentence.get_field(6));
    }
    // Parse track true north.
    if(sentence.has_field(7))
    {
        // Parse degrees and convert to radians.
        message_gnss_track.heading = std::stod(sentence.get_field(7)) * M_PI / 180.0;
    }
    // Publish message.
    driver::m_publisher_gnss_track.publish(message_gnss_track);
}