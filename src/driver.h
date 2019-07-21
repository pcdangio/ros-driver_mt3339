#ifndef DRIVER_H
#define DRIVER_H

#include <serial/serial.h>

class driver
{
public:


    driver(std::string port, unsigned int baud_rate);
    ~driver();

private:
    serial::Serial* m_port;
};

#endif // DRIVER_H
