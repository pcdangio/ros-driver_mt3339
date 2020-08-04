#include "driver.h"

#include <ctime>
#include <chrono>
#include <stdexcept>

// CONSTRUCTORS
driver::driver(std::string port, unsigned int baud_rate)
{
    // Initialize PMTK response trackers.
    driver::m_last_ack.is_set = false;
    driver::m_last_txt.is_set = false;

    // Open the serial port.
    // Use interbyte timeout since partial NMEA strings can be sent.
    driver::m_port = new serial::Serial(port, baud_rate, serial::Timeout(10, 100, 0, 100, 0));

    // Start the read thread.
    driver:f_stop_requested = false;
    driver::m_thread = boost::thread(&driver::read_thread, this);
}
driver::~driver()
{
    // Stop the read thread.
    driver::f_stop_requested = true;
    driver::m_thread.join();

    // Close and clean up the serial port.
    driver::m_port->close();
    delete driver::m_port;
}

// CONFIGURATION
bool driver::test_connection()
{
    // Create a TXT message to send to the MT3339.
    std::vector<std::string> fields;
    fields.push_back("test_connection");
    message msg("PMTK", "011", fields);

    // Send the message.
    driver::send_message(msg);

    // Retrieve the TXT message and check if it matches.
    std::string retrieved_text;
    if(driver::get_txt(retrieved_text) && retrieved_text == "test_connection")
    {
        return true;
    }

    // If this point reached, either:
    // - No TXT message was sent by the MT3339
    // - The last TXT message does not match what was sent.
    return false;
}
void driver::set_baud(unsigned int baud_rate)
{
    // Build message to send.
    std::vector<std::string> fields;
    fields.push_back(std::to_string(baud_rate));
    message msg("PMTK", "251", fields);

    // Send message.
    driver::send_message(msg);

    // Do not try and receive an ACK as the baud rate is changed.
}
bool driver::set_nmea_update_rate(unsigned int milliseconds, ack_t* ack)
{
    // Build the message.
    std::vector<std::string> fields;
    fields.push_back(std::to_string(milliseconds));
    message msg("PMTK", "220", fields);

    // Send the message.
    driver::send_message(msg);

    // Retrieve the ack.
    return driver::get_ack("220", ack);
}
bool driver::set_nmea_output(ack_t* ack)
{
    // Build message according to what callbacks are set.
    std::vector<std::string> fields;
    fields.resize(19, "0");
    if(driver::m_callback_rmc)
    {
        fields[1] = "1";
    }
    if(driver::m_callback_vtg)
    {
        fields[2] = "1";
    }
    if(driver::m_callback_gga)
    {
        fields[3] = "1";
    }
    if(driver::m_callback_gsa)
    {
        fields[4] = "1";
    }
    message msg("PMTK", "314", fields);

    // Send message.
    driver::send_message(msg);
    
    // Retrieve the ack.
    return driver::get_ack("314", ack);
}

// IO METHODS
void driver::send_message(const message& msg)
{
    // Write the message to the MT3339
    driver::m_port->write(msg.p_nmea_sentence());
}

// READ
void driver::read_thread()
{
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

        // Convert string into message.
        message nmea(buffer);

        // Validate the message.
        if(!nmea.validate_checksum())
        {
            continue;
        }

        // Handle the message based on talker and type.
        std::string talker = nmea.p_talker();
        std::string type = nmea.p_type();
        if(talker == "PMTK")
        {
            if(type == "001")
            {
                driver::handle_ack(nmea);
            }
            else if(type == "011")
            {
                driver::handle_txt(nmea);
            }
        }
        else if(talker == "GP")
        {
            if(type == "GGA")
            {
                driver::handle_gga(nmea);
            }
            else if(type == "GSA")
            {
                driver::handle_gsa(nmea);
            }
            else if(type == "RMC")
            {
                driver::handle_rmc(nmea);
            }
            else if(type == "VTG")
            {
                driver::handle_vtg(nmea);
            }
        }
    }
}

// HANDLERS
void driver::handle_ack(const message& msg)
{
    // Lock ACK mutex.
    driver::m_mutex_ack.lock();

    // Set last ack.
    driver::m_last_ack.is_set = true;
    auto fields = msg.p_data();
    driver::m_last_ack.command = fields.at(0);
    driver::m_last_ack.ack = static_cast<driver::ack_t>(std::stoi(fields.at(1)));

    // Unlock the mutex.
    driver::m_mutex_ack.unlock();
}
void driver::handle_txt(const message& msg)
{
    // Lock TXT mutex.
    driver::m_mutex_txt.lock();

    // Set last txt.
    driver::m_last_txt.is_set = true;
    auto fields = msg.p_data();
    driver::m_last_txt.text = fields.at(0);

    // Unlock mutex.
    driver::m_mutex_txt.unlock();
}
void driver::handle_gga(const message& msg)
{
    // Check if the callback is set.
    if(driver::m_callback_gga)
    {
        // Create a new GGA instance.
        auto gga = std::make_shared<nmea::gga>();

        // Get message fields.
        auto fields = msg.p_data();        

        // Parse UTC time of day from first field.
        gga->utc_time_of_day = std::stod(fields[0].substr(0, 2)) * 3600.0;
        gga->utc_time_of_day += std::stod(fields[0].substr(2, 2)) * 60.0;
        gga->utc_time_of_day += std::stod(fields[0].substr(4));

        // Parse latitude.
        if(!fields[1].empty())
        {
            gga->latitude = std::stod(fields[1].substr(0,2));
            gga->latitude += std::stod(fields[1].substr(2)) / 60.0;
            if(fields[2] == "S")
            {
                gga->latitude *= -1;
            }
        }
        else
        {
            gga->latitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse longitude.
        if(!fields[3].empty())
        {
            gga->longitude = std::stod(fields[3].substr(0,2));
            gga->longitude += std::stod(fields[3].substr(2)) / 60.0;
            if(fields[4] == "W")
            {
                gga->longitude *= -1;
            }
        }
        else
        {
            gga->longitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse fix quality.
        gga->fix_quality = static_cast<nmea::gga::fix_quality_t>(std::stoi(fields[5]));

        // Parse satellite count.
        if(!fields[6].empty())
        {
            gga->satellite_count = std::stoul(fields[6]);
        }

        // Parse HDOP.
        if(!fields[7].empty())
        {
            gga->hdop = std::stod(fields[7]);
        }

        // Parse Altitude
        if(!fields[8].empty())
        {
            gga->altitude = std::stod(fields[8]);
        }
        else
        {
            gga->altitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse Geoid Height
        if(!fields[10].empty())
        {
            gga->geoid_height = std::stod(fields[10]);
        }
        else
        {
            gga->geoid_height = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse DGPS age.
        if(!fields[12].empty())
        {
            gga->dgps_age = std::stod(fields[12]);
        }
        else
        {
            gga->dgps_age = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse DGPS station ID.
        if(!fields[13].empty())
        {
            gga->dgps_id = std::stoul(fields[13]);
        }

        // Raise the callback.
        driver::m_callback_gga(gga);
    }
}
void driver::handle_gsa(const message& msg)
{
    // Check if callback is set.
    if(driver::m_callback_gsa)
    {
        // Create a new GSA instance.
        auto gsa = std::make_shared<nmea::gsa>();

        // Get fields from the message.
        auto fields = msg.p_data();

        // Parse mode.
        std::string mode_indicator = fields[0];
        if(mode_indicator == "A")
        {
            gsa->mode = nmea::gsa::mode_t::AUTOMATIC;
        }
        else if(mode_indicator == "M")
        {
            gsa->mode = nmea::gsa::mode_t::MANUAL;
        }

        // Parse fix type.
        gsa->fix_type = static_cast<nmea::gsa::fix_type_t>(std::stoi(fields[1])-1);

        // Read in satellite PRNs.
        for(uint32_t i = 0; i < 12; ++i)
        {
            if(!fields[2+i].empty())
            {
                gsa->satellites.push_back(std::stoul(fields[2+i]));
            }
        }

        // Parse PDOP
        if(!fields[14].empty())
        {
            gsa->pdop = std::stod(fields[14]);
        }
        else
        {
            gsa->pdop = std::numeric_limits<float>::quiet_NaN();
        }

        // Parse HDOP
        if(!fields[15].empty())
        {
            gsa->hdop = std::stod(fields[15]);
        }
        else
        {
            gsa->hdop = std::numeric_limits<float>::quiet_NaN();
        }

        // Parse VDOP
        if(!fields[16].empty())
        {
            gsa->vdop = std::stod(fields[16]);
        }
        else
        {
            gsa->vdop = std::numeric_limits<float>::quiet_NaN();
        }

        // Raise callback for GSA message.
        driver::m_callback_gsa(gsa);
    }
}
void driver::handle_rmc(const message& msg)
{
    // Check if callback exists.
    if(driver::m_callback_rmc)
    {
        // Create RMC instance.
        auto rmc = std::make_shared<nmea::rmc>();

        // Read message fields.
        auto fields = msg.p_data();

        // Time Field (idx 0): hhmmss.sss
        // Date Field (idx 8): ddmmyy
        std::tm time_struct;
        // Convert date + hours + minutes to double.
        time_struct.tm_hour = std::stoi(fields[0].substr(0, 2));
        time_struct.tm_min = std::stoi(fields[0].substr(2, 2));
        time_struct.tm_mday = std::stoi(fields[8].substr(0,2));
        time_struct.tm_mon = std::stoi(fields[8].substr(2,2)) - 1;
        time_struct.tm_year = std::stoi(fields[8].substr(4, 2)) + 100;
        rmc->utc_time = static_cast<double>(timegm(&time_struct));
        // Parse and add in seconds.
        rmc->utc_time += std::stod(fields[0].substr(4));

        // Parse status.
        std::string status_indicator = fields[1];
        if(status_indicator == "A")
        {
            rmc->status = nmea::rmc::status_t::ACTIVE;
        }
        else if(status_indicator == "v")
        {
            rmc->status = nmea::rmc::status_t::VOID;
        }

        // Parse latitude.
        if(!fields[2].empty())
        {
            rmc->latitude = std::stod(fields[2].substr(0,2));
            rmc->latitude += std::stod(fields[2].substr(2)) / 60.0;
            if(fields[3] == "S")
            {
                rmc->latitude *= -1;
            }
        }
        else
        {
            rmc->latitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse longitude.
        if(!fields[4].empty())
        {
            rmc->longitude = std::stod(fields[4].substr(0,2));
            rmc->longitude += std::stod(fields[4].substr(2)) / 60.0;
            if(fields[5] == "W")
            {
                rmc->longitude *= -1;
            }
        }
        else
        {
            rmc->longitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse ground speed.
        if(!fields[6].empty())
        {
            rmc->ground_speed = std::stod(fields[6]);
        }
        else
        {
            rmc->ground_speed = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse track true north.
        if(!fields[7].empty())
        {
            rmc->track_true = std::stod(fields[7]);
        }
        else
        {
            rmc->track_true = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse magnetic variation.
        if(!fields[9].empty())
        {
            rmc->magnetic_variation = std::stod(fields[9]);
        }
        else
        {
            rmc->magnetic_variation = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse the mode.
        std::string mode_indicator = fields[10];
        if(mode_indicator == "N")
        {
            rmc->mode = nmea::rmc::mode_t::INVALID;
        }
        else if(mode_indicator == "A")
        {
            rmc->mode = nmea::rmc::mode_t::AUTONOMOUS;
        }
        else if(mode_indicator == "D")
        {
            rmc->mode = nmea::rmc::mode_t::DIFFERENTIAL;
        }
        else if(mode_indicator == "E")
        {
            rmc->mode = nmea::rmc::mode_t::ESTIMATED;
        }
        else if(mode_indicator == "M")
        {
            rmc->mode = nmea::rmc::mode_t::MANUAL;
        }

        // Raise callback.
        driver::m_callback_rmc(rmc);
    }
}

// LAST MESSAGE
bool driver::get_ack(const std::string& command, ack_t* ack)
{
    // Set up output.
    bool retrieved = false;

    // Loop until retrieved or timeout of 100ms.
    std::chrono::duration<int32_t, std::chrono::milliseconds> timeout(100);
    auto start_time = std::chrono::steady_clock::now();
    while(!retrieved && (std::chrono::steady_clock::now() - start_time) <= timeout)
    {
        // Lock mutex.
        driver::m_mutex_ack.lock();

        // Check if last_ack is set.
        if(driver::m_last_ack.is_set)
        {
            // Check if last ack is for the requested command.
            if(driver::m_last_ack.command == command)
            {
                // Retrieve the ack.
                if(ack)
                {
                    *ack = driver::m_last_ack.ack;
                }
                driver::m_last_ack.is_set = false;
                retrieved = true;
            }
        }

        // Unlock the mutex.
        driver::m_mutex_ack.unlock();

        // Sleep if still waiting.
        if(!retrieved)
        {
            usleep(5000);
        }        
    }    

    return retrieved;
}
bool driver::get_txt(std::string& text)
{
    // Set up output.
    bool retrieved = false;

    // Loop until retrieved or timeout of 100ms.
    std::chrono::duration<int32_t, std::chrono::milliseconds> timeout(100);
    auto start_time = std::chrono::steady_clock::now();
    while(!retrieved && (std::chrono::steady_clock::now() - start_time) <= timeout)
    {
        // Lock mutex.
        driver::m_mutex_txt.lock();

        // Check if last_ack is set.
        if(driver::m_last_txt.is_set)
        {
            // Retrieve the text.
            text = driver::m_last_txt.text;
            driver::m_last_txt.is_set = false;
            retrieved = true;
        }

        // Unlock the mutex.
        driver::m_mutex_txt.unlock();

        // Sleep if still waiting.
        if(!retrieved)
        {
            usleep(5000);
        }        
    }    

    return retrieved;
}