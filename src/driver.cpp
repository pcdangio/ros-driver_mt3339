#include "driver.h"

driver::driver(std::string port, unsigned int baud_rate)
{
    driver::m_port = new serial::Serial(port, baud_rate);
}
driver::~driver()
{
    delete driver::m_port;
}
