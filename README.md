# driver_mt3339

## Overview

This package includes driver software for the [MT3339] GPS Receiver.

**Keywords:** mt3339 gps driver

### License

The source code is released under a [MIT license](LICENSE).

**Author: Paul D'Angio<br />
Maintainer: Paul D'Angio, pcdangio@gmail.com**

The driver_mt3339 package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [serial](http://wiki.ros.org/serial) (ROS serial package)
- [sensor_msgs](http://wiki.ros.org/sensor_msgs) (ROS sensor_msgs)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        cd catkin_workspace/src
        git clone https://github.com/pcdangio/ros-driver_mt3339.git driver_mt3339
        cd ../
        catkin_make

## Usage

Run the driver with the following command:

        rosrun driver_mt3339 node

## Nodes

### node

A driver for interacting with the [MT3339] GPS.  Enables configuration and reading of NMEA data.


#### Published Topics
* **`gps/position`** ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

        The GPS position measured by the sensor.

* **`gps/time`** ([sensor_msgs/TimeReference](http://docs.ros.org/api/sensor_msgs/html/msg/TimeReference.html))

        The current UTC time measured by the sensor.


#### Runtime Parameters

* **`~/serial_port`** (string, default: /dev/ttyAMA0)

        The serial port connected to the sensor.

* **`~/baud_rate`** (uint32, default: 9600)

        The baud rate to use for serial communication with the sensor.

* **`~/scan_rate`** (double, default: 15)

        The rate in Hz at which to scan the serial port for new NMEA messages.

* **`~/uere`** (double, default: 6.74)

        The User Equivalent Range Error (UERE) representing the total pseudorange error budget.  This is typically 6.74 for C/A, and 6.0 for P(Y).

#### Configuration Parameters

These parameters should only be changed when needed, and not set prior to every run.  These parameters will be updated in the sensor's flash memory and will be preserved over power cycles.

* **`~/update_baud`** (int, default: -1)

        Changes the baud rate of the MT3339 GPS.  Possible values are:
        4800 bps
        9600 bps
        14400 bps
        19200 bps
        38400 bps
        57600 bps
        115200 bps
        The default value of -1 instructs the node to ignore updating the baud rate.

* **`~/update_nmea_rate`** (unsigned int, default: -1)

        Changes the position update rate of the MT3339 GPS in milliseconds.
        The acceptable rates are between 100ms and 10,000ms.
        Position fixes are output once every period specified, so 100ms = 10Hz.
        The default value of -1 instructs the node to ignore updating the update rate.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_mt3339/issues).


[ROS]: http://www.ros.org
[MT3339]: https://cdn-shop.adafruit.com/datasheets/GlobalTop-FGPMMOPA6H-Datasheet-V0A.pdf
