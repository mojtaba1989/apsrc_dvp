# ACM Dynamic Velocity Profile Modification

This directory contains all the software for enabling dynamic velocity profile modification within autoware.
The software is broken down into two ROS packages:  
1. `dvp_waypoint_replanner`: This package is responsible for integrating into the Autoware stack and receiving velocity profiles from the MABx.
This package also contains a node for sending out tracked objects UDP packets.
2. `dvp_validation`: This package is responsible for acting in the place of the MABx, sending velocity profiles and receiving localization/status replies.
It also receives incoming tracked objects packets and visualizes them.

See the "README.md" files in both the `dvp_waypoint_replanner` and `dvp_validation` directories/packages for more details.

## Validation Scenarios

In order to validate the functionality of the software and its installation, a list of validation scenarios was created to ensure the software works as expected under different circumstances.
See "Validation.md" for more details.

## UDP package definitions

The following is the UDP package definitions for the interface between the dSpace MicroAutoBox and the Autoware system.

NOTE: Messages are subject to change.

The MABx will act as a UDP client and connect to the Spectra Computer on port 1551.
A UDP server will be running on the Spectra and bound to port 1551.
The UDP server will respond to whichever port was used to contact it in the first place.
Localization messages will only be sent after first receiving a Velocity Profile message.
Velocity Profile messages should not be sent at a rate faster than 10 Hz.

```
# Velocity Profile UDP Packet (total length = 118 bytes)
uint8 msg_id                      # 0 to 127 and repeating, rolling counter
uint8 status                      # 0 = test aborted/inactive, 1 = test active
uint16 first_global_waypoint_id   # The global waypoint id of the first waypoint in the 50-waypoint set
uint16[50] target_velocities      # The target velocities in (mm/s)
10 x uint8                        # Reserved for future use
uint32 crc                        # 32-bit CRC

# Localization UDP Packet (total length = 24 bytes)
uint8 msg_id                      # 0 to 127, should match the msg_id from the velocity profile packet
bool data_valid                   # Whether or not the following data is valid/ready
bool path_tracking_enabled        # DBW enable status, 0 = disengaged manual driving, 1 = autonomy engaged
bool velocity_profile_enabled     # True if provided velocity profile is being used, False if default speed from original global path is being followed
uint16 closest_global_waypoint_id # The global waypoint id of the closest waypoint to the rear axle of the vehicle
uint16 target_global_velocity     # The target velocity of the possibly modified global path at the closest waypoint (mm/s)
uint16 current_velocity           # The current velocity of the vehicle in (mm/s)
10 x uint8                        # Reserved for future use
uint32 crc                        # 32-bit CRC

# Tracked Objects Packet (total length = 465 bytes)
uint8 num_objects                 # The number of valid objects in the packet
int16[30] x_pos                   # The relative position of the object (cm)
int16[30] y_pos                   # The relative position of the object (cm)
int16[30] speed                   # The speed of the object (mm/s)
int16[30] heading                 # The relative heading of the object (rad * 100)
uint8[30] classification          # The classification of the object (1 = unknown, 2 = car, 3 = pedestrian, 4 = bicycle)
uint16[30] size                   # The largest rectangular dimension, usually length of vehicle (mm), object height is not considered
uint32[30] id                     # Id of the object
10 x uint8                        # Reserved for future use
uint32 crc                        # 32-bit CRC
```

## Global Waypoint File Format  

Simple csv file with the following header/columns:

wp_id, x, y, z, lat, lon, yaw, velocity, change_flag  

Velocity is in km/h


## Assumptions

- The global speed profile encapsulated in the global waypoint file is already considered safe/appropriate. No additional checks for speeds around turns will be performed, since they are assumed to be safe already.
- The global speed profile from the file and any modifications to it (via MABx) will be followed as closely as possible. It will only be overridden by deceleration for stop signs, stop lights, and objects in the path.
