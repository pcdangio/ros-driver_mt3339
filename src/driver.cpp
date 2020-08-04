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
    // Create a TXT sentence to send to the MT3339.
    nmea::sentence sentence("PMTK", "011", 1);
    sentence.set_field(0, "test_connection");

    // Send the sentence.
    driver::send_sentence(sentence);

    // Retrieve the TXT sentence and check if it matches.
    std::string retrieved_text;
    if(driver::get_txt(retrieved_text) && retrieved_text == "test_connection")
    {
        return true;
    }

    // If this point reached, either:
    // - No TXT sentence was sent by the MT3339
    // - The last TXT sentence does not match what was sent.
    return false;
}
void driver::set_baud(unsigned int baud_rate)
{
    // Build sentence to send.
    nmea::sentence sentence("PMTK", "251", 1);
    sentence.set_field(0, std::to_string(baud_rate));

    // Send sentence.
    driver::send_sentence(sentence);

    // Do not try and receive an ACK as the baud rate is changed.
}
bool driver::set_nmea_update_rate(unsigned int milliseconds, ack_t* ack)
{
    // Build the sentence.
    nmea::sentence sentence("PMTK", "220", 1);
    sentence.set_field(0, std::to_string(milliseconds));

    // Send the sentence.
    driver::send_sentence(sentence);

    // Retrieve the ack.
    return driver::get_ack("220", ack);
}
bool driver::set_nmea_output(ack_t* ack)
{
    // Build sentence according to what callbacks are set.
    nmea::sentence sentence("PMTK", "314", 19);
    // Initialize all fields to zero.
    for(uint32_t i = 0; i < 19; ++i)
    {
        sentence.set_field(i, "0");
    }
    // Set type fields based on callbacks.
    if(driver::m_callback_rmc)
    {
        sentence.set_field(1, "1");
    }
    if(driver::m_callback_gga)
    {
        sentence.set_field(3, "1");
    }
    if(driver::m_callback_gsa)
    {
        sentence.set_field(4, "1");
    }

    // Send sentence.
    driver::send_sentence(sentence);
    
    // Retrieve the ack.
    return driver::get_ack("314", ack);
}

// IO METHODS
void driver::send_sentence(const nmea::sentence& sentence)
{
    // Write the sentence to the MT3339
    driver::m_port->write(sentence.p_nmea_sentence());
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
            else if(type == "011")
            {
                driver::handle_txt(sentence);
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
}

// HANDLERS
void driver::handle_ack(const nmea::sentence& sentence)
{
    // Lock ACK mutex.
    driver::m_mutex_ack.lock();

    // Set last ack.
    driver::m_last_ack.is_set = true;
    driver::m_last_ack.command = sentence.get_field(0);
    driver::m_last_ack.ack = static_cast<driver::ack_t>(std::stoi(sentence.get_field(1)));

    // Unlock the mutex.
    driver::m_mutex_ack.unlock();
}
void driver::handle_txt(const nmea::sentence& sentence)
{
    // Lock TXT mutex.
    driver::m_mutex_txt.lock();

    // Set last txt.
    driver::m_last_txt.is_set = true;
    driver::m_last_txt.text = sentence.get_field(0);

    // Unlock mutex.
    driver::m_mutex_txt.unlock();
}
void driver::handle_gga(const nmea::sentence& sentence)
{
    // Check if the callback is set.
    if(driver::m_callback_gga)
    {
        // Create a new GGA instance.
        auto gga = std::make_shared<nmea::gga>();     

        // Parse UTC time of day from first field.
        gga->utc_time_of_day = std::stod(sentence.get_field(0).substr(0, 2)) * 3600.0;
        gga->utc_time_of_day += std::stod(sentence.get_field(0).substr(2, 2)) * 60.0;
        gga->utc_time_of_day += std::stod(sentence.get_field(0).substr(4));

        // Parse latitude.
        if(sentence.has_field(1))
        {
            gga->latitude = std::stod(sentence.get_field(1).substr(0,2));
            gga->latitude += std::stod(sentence.get_field(1).substr(2)) / 60.0;
            if(sentence.get_field(2) == "S")
            {
                gga->latitude *= -1;
            }
        }
        else
        {
            gga->latitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse longitude.
        if(sentence.has_field(3))
        {
            gga->longitude = std::stod(sentence.get_field(3).substr(0,2));
            gga->longitude += std::stod(sentence.get_field(3).substr(2)) / 60.0;
            if(sentence.get_field(4) == "W")
            {
                gga->longitude *= -1;
            }
        }
        else
        {
            gga->longitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse fix quality.
        gga->fix_quality = static_cast<nmea::gga::fix_quality_t>(std::stoi(sentence.get_field(5)));

        // Parse satellite count.
        if(sentence.has_field(6))
        {
            gga->satellite_count = std::stoul(sentence.get_field(6));
        }

        // Parse HDOP.
        if(sentence.has_field(7))
        {
            gga->hdop = std::stod(sentence.get_field(7));
        }

        // Parse Altitude
        if(sentence.has_field(8))
        {
            gga->altitude = std::stod(sentence.get_field(8));
        }
        else
        {
            gga->altitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse Geoid Height
        if(sentence.has_field(10))
        {
            gga->geoid_height = std::stod(sentence.get_field(10));
        }
        else
        {
            gga->geoid_height = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse DGPS age.
        if(sentence.has_field(12))
        {
            gga->dgps_age = std::stod(sentence.get_field(12));
        }
        else
        {
            gga->dgps_age = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse DGPS station ID.
        if(sentence.has_field(13))
        {
            gga->dgps_id = std::stoul(sentence.get_field(13));
        }

        // Raise the callback.
        driver::m_callback_gga(gga);
    }
}
void driver::handle_gsa(const nmea::sentence& sentence)
{
    // Check if callback is set.
    if(driver::m_callback_gsa)
    {
        // Create a new GSA instance.
        auto gsa = std::make_shared<nmea::gsa>();

        // Parse mode.
        std::string mode_indicator = sentence.get_field(0);
        if(mode_indicator == "A")
        {
            gsa->mode = nmea::gsa::mode_t::AUTOMATIC;
        }
        else if(mode_indicator == "M")
        {
            gsa->mode = nmea::gsa::mode_t::MANUAL;
        }

        // Parse fix type.
        gsa->fix_type = static_cast<nmea::gsa::fix_type_t>(std::stoi(sentence.get_field(1))-1);

        // Read in satellite PRNs.
        for(uint32_t i = 0; i < 12; ++i)
        {
            if(sentence.has_field(2+i))
            {
                gsa->satellites.push_back(std::stoul(sentence.get_field(2+i)));
            }
        }

        // Parse PDOP
        if(sentence.has_field(14))
        {
            gsa->pdop = std::stod(sentence.get_field(14));
        }
        else
        {
            gsa->pdop = std::numeric_limits<float>::quiet_NaN();
        }

        // Parse HDOP
        if(sentence.has_field(15))
        {
            gsa->hdop = std::stod(sentence.get_field(15));
        }
        else
        {
            gsa->hdop = std::numeric_limits<float>::quiet_NaN();
        }

        // Parse VDOP
        if(sentence.has_field(16))
        {
            gsa->vdop = std::stod(sentence.get_field(16));
        }
        else
        {
            gsa->vdop = std::numeric_limits<float>::quiet_NaN();
        }

        // Raise callback for GSA sentence.
        driver::m_callback_gsa(gsa);
    }
}
void driver::handle_rmc(const nmea::sentence& sentence)
{
    // Check if callback exists.
    if(driver::m_callback_rmc)
    {
        // Create RMC instance.
        auto rmc = std::make_shared<nmea::rmc>();

        // Time Field (idx 0): hhmmss.sss
        // Date Field (idx 8): ddmmyy
        std::tm time_struct;
        // Convert date + hours + minutes to double.
        std::string field_time = sentence.get_field(0);
        std::string field_date = sentence.get_field(8);
        time_struct.tm_hour = std::stoi(field_time.substr(0, 2));
        time_struct.tm_min = std::stoi(field_time.substr(2, 2));
        time_struct.tm_mday = std::stoi(field_date.substr(0,2));
        time_struct.tm_mon = std::stoi(field_date.substr(2,2)) - 1;
        time_struct.tm_year = std::stoi(field_date.substr(4, 2)) + 100;
        rmc->utc_time = static_cast<double>(timegm(&time_struct));
        // Parse and add in seconds.
        rmc->utc_time += std::stod(field_time.substr(4));

        // Parse status.
        std::string status_indicator = sentence.get_field(1);
        if(status_indicator == "A")
        {
            rmc->status = nmea::rmc::status_t::ACTIVE;
        }
        else if(status_indicator == "v")
        {
            rmc->status = nmea::rmc::status_t::VOID;
        }

        // Parse latitude.
        if(sentence.has_field(2))
        {
            std::string field_latitude = sentence.get_field(2);
            rmc->latitude = std::stod(field_latitude.substr(0,2));
            rmc->latitude += std::stod(field_latitude.substr(2)) / 60.0;
            if(sentence.get_field(3) == "S")
            {
                rmc->latitude *= -1;
            }
        }
        else
        {
            rmc->latitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse longitude.
        if(sentence.has_field(4))
        {
            std::string field_longitude = sentence.get_field(4);
            rmc->longitude = std::stod(field_longitude.substr(0,2));
            rmc->longitude += std::stod(field_longitude.substr(2)) / 60.0;
            if(sentence.get_field(5) == "W")
            {
                rmc->longitude *= -1;
            }
        }
        else
        {
            rmc->longitude = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse ground speed.
        if(sentence.has_field(6))
        {
            rmc->ground_speed = std::stod(sentence.get_field(6));
        }
        else
        {
            rmc->ground_speed = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse track true north.
        if(sentence.has_field(7))
        {
            rmc->track_true = std::stod(sentence.get_field(7));
        }
        else
        {
            rmc->track_true = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse magnetic variation.
        if(sentence.has_field(9))
        {
            rmc->magnetic_variation = std::stod(sentence.get_field(9));
        }
        else
        {
            rmc->magnetic_variation = std::numeric_limits<double>::quiet_NaN();
        }

        // Parse the mode.
        std::string mode_indicator = sentence.get_field(10);
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

// LAST sentence
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