#include "driver.h"

#include <ctime>
#include <chrono>

// CONSTRUCTORS
driver::driver(std::string port, unsigned int baud_rate)
{
    // Open the serial port.
    // Use interbyte timeout since partial NMEA strings can be sent.
    driver::m_port = new serial::Serial(port, baud_rate, serial::Timeout(10, 100));

    // Initialize data ready flag.
    driver::m_data_ready = 0;
}
driver::~driver()
{
    driver::m_port->close();
    delete driver::m_port;
}

// CONFIGURATION
bool driver::set_baud(unsigned int baud_rate)
{
    std::vector<std::string> fields;
    fields.push_back(std::to_string(baud_rate));
    message msg("PMTK", "251", fields);

    return driver::send_message(msg);
}
bool driver::set_nmea_update_rate(unsigned int milliseconds)
{
    std::vector<std::string> fields;
    fields.push_back(std::to_string(milliseconds));
    message msg("PMTK", "220", fields);

    return driver::send_message(msg);
}
bool driver::set_nmea_output()
{
    std::vector<std::string> fields;
    fields.push_back("0");  // GLL
    fields.push_back("1");  // RMC
    fields.push_back("0");  // VTG
    fields.push_back("1");  // GGA
    fields.push_back("1");  // GSA
    fields.push_back("0");  // GSV
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // Reserved
    fields.push_back("0");  // CHN
    message msg("PMTK", "314", fields);

    return driver::send_message(msg);
}

// IO METHODS
bool driver::send_message(message msg)
{
    // Write the message to the MT3339
    driver::m_port->write(msg.p_nmea_sentence());

    // Attempt to read ACK with timeout.
    // Create timestamp for tracking read timeout.
    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

    // Read and handle messages until ACK received.
    message* response = nullptr;
    while(!response)
    {
        // Read next message.
        response = driver::read_message();

        // Check if message is valid.
        if(response)
        {
            // Check if message is PMTK_ACK.
            if(response->p_talker().compare("PMTK") == 0 && response->p_type().compare("001") == 0)
            {
                // ACK message.
                // Read fields.
                std::vector<std::string> fields = response->p_data();
                // Check if acked command matches the sent command.
                if(fields.at(0).compare(msg.p_type()) == 0)
                {
                    // Check the ack type.
                    if(fields.at(1).compare("3") == 0)
                    {
                        delete response;
                        return true;
                    }
                }
            }
            // Check if message is from a GP talker.
            else if(response->p_talker().compare("GP") == 0)
            {
                // GP message.
                driver::handle_gp(response);
            }

            // Clean up response.
            delete response;
        }

        // Check timeout in milliseconds.
        long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        if(elapsed > 100)
        {
            break;
        }
    }

    return false;
}
message* driver::read_message()
{
    // Read the next line NMEA sentence.
    std::string buffer;
    driver::m_port->readline(buffer, 200, "\r\n");

    // Convert string into message.
    message* output = new message(buffer);

    // Validate the message.
    if(output->validate_checksum() == false)
    {
        delete output;
        output = nullptr;
    }

    // Return message.
    return output;
}
void driver::handle_gp(const message *msg)
{
    // Store local reference to message type.
    std::string msg_type = msg->p_type();

    if(msg_type.compare("GGA") == 0)
    {
        // Get Lat/Long/Alt
        std::vector<std::string> fields = msg->p_data();

        // Lat Field (idx 1): ddmm.mmmm
        // Read degrees portion of field.
        driver::m_current_data.latitude = std::stod(fields.at(1).substr(0, 2));
        // Read minutes and add to degrees.
        driver::m_current_data.latitude += std::stod(fields.at(1).substr(2)) / 60.0;
        // Read N/S field (idx 2) and modify latitude as needed.
        if(fields.at(2).compare("S") == 0)
        {
            driver::m_current_data.latitude *= -1.0;
        }

        // Long Field (idx 3): dddmm.mmmm
        // Read degrees portion of field.
        driver::m_current_data.longitude = std::stod(fields.at(3).substr(0, 3));
        // Read minutes and add to degrees.
        driver::m_current_data.longitude += std::stod(fields.at(3).substr(3)) / 60.0;
        // Read N/S field (idx 4) and modify latitude as needed.
        if(fields.at(4).compare("W") == 0)
        {
            driver::m_current_data.longitude *= -1.0;
        }

        // Alt Field (idx 8)
        driver::m_current_data.altitude = std::stod(fields.at(8));

        // Set GGA ready in flag.
        driver::m_data_ready |= 0x01;
    }
    else if(msg_type.compare("GSA") == 0)
    {
        // Get Fix, HDOP, VDOP
        std::vector<std::string> fields = msg->p_data();

        // Fix Field (idx 1)
        driver::m_current_data.fix = static_cast<driver::data::fix_type>(std::stoi(fields.at(1)));

        // HDOP Field (idx 15)
        driver::m_current_data.hdop = std::stod(fields.at(15));

        // VDOP Field (idx 16)
        driver::m_current_data.vdop = std::stod(fields.at(16));

        // Set GSA ready in flag.
        driver::m_data_ready |= 0x02;
    }
    else if(msg_type.compare("RMC") == 0)
    {
        // Get Date, Time
        std::vector<std::string> fields = msg->p_data();

        // Time Field (idx 0): hhmmss.sss
        // Date Field (idx 8): ddmmyy
        std::tm time_struct;
        // Convert date + hours + minutes to double.
        time_struct.tm_hour = std::stoi(fields.at(0).substr(0, 2));
        time_struct.tm_min = std::stoi(fields.at(0).substr(2, 2));
        time_struct.tm_mday = std::stoi(fields.at(8).substr(0,2));
        time_struct.tm_mon = std::stoi(fields.at(8).substr(2,2)) - 1;
        time_struct.tm_year = std::stoi(fields.at(8).substr(4, 2)) + 100;
        driver::m_current_data.utc_time = static_cast<double>(std::mktime(&time_struct));

        // Parse and add in seconds.
        driver::m_current_data.utc_time += std::stod(fields.at(0).substr(4));

        // Set RMC ready in flag.
        driver::m_data_ready |= 0x04;
    }

    // Check if all data is ready.
    if(driver::m_data_ready == 0x07)
    {
        // Raise data ready callback.
        driver::m_callback(driver::m_current_data);

        // Reset data ready flag.
        driver::m_data_ready = 0x00;
    }
}

// MISC METHODS
void driver::attach_data_callback(std::function<void(data)> callback)
{
    driver::m_callback = callback;
}
void driver::spin_once()
{
    // Read all messages until timeout.
    while(true)
    {
        // Attempt to read next message from RX buffer.
        message* msg = driver::read_message();
        // Check if message was read.
        if(msg)
        {
            // Handle message if from GP talker.
            if(msg->p_talker().compare("GP") == 0)
            {
                driver::handle_gp(msg);
            }
            // Clean up message.
            delete msg;
        }
        else
        {
            // No messages left in RX buffer. Break loop.
            break;
        }
    }
}
