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
- [sensor_msgs_ext](https://github.com/pcdangio/ros-sensor_msgs_ext) (ROS sensor_msgs)

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
* **`gnss/fix`** ([sensor_msgs_ext/gnss_fix](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/gnss_fix.msg))

        The status of the sensor's GNSS fix.

* **`gnss/position`** ([sensor_msgs_ext/gnss_position](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/gnss_position.msg))

        The current GNSS position. Only published while a fix is available.

* **`gnss/track`** ([sensor_msgs_ext/gnss_track](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/gnss_track.msg))

        The current GNSS track. Only published while a fix is available.

* **`gnss/time`** ([sensor_msgs_ext/time_reference](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/time_reference.msg))

        The current UTC time measured by the sensor. Only published while a fix is available.


#### Parameters

* **`~/serial_port`** (string, default: /dev/ttyAMA0)

        The serial port connected to the sensor.

* **`~/baud_rate`** (uint32, default: 38400)

        The baud rate to use for serial communication with the sensor.

* **`~/connection_settle_time`** (uint32, default: 300)

        The number of milliseconds to wait after opening the serial port and test connectivity to the sensor.

* **`~/timeout`** (uint32, default: 300)

        The number of milliseconds to wait for message responses from the sensor.

* **`~/update_rate`** (uint32, default: 100)

        Changes the position update rate of the MT3339 GPS in milliseconds.
        The acceptable rates are between 100ms and 10,000ms.
        Position fixes are output once every period specified, so 100ms = 10Hz.

* **`~/frame_id`** (string, default: mt3339)

        The name of the coordinate frame that the sensor is located in.

* **`~/uere`** (double, default: 6.74)

        The User Equivalent Range Error (UERE) representing the total pseudorange error budget.  This is typically 6.74 for C/A, and 6.0 for P(Y).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_mt3339/issues).


[ROS]: http://www.ros.org
[MT3339]: https://cdn-shop.adafruit.com/datasheets/GlobalTop-FGPMMOPA6H-Datasheet-V0A.pdf
