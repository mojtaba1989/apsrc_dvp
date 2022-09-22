# dvp_validation

This ROS package provides a means of validating the Autoware.AI modifications required for dynamic velocity profile modification.
The dvp_validation node emulates the user's software by providing the same API and exchanging dummy/test data with Autoware.AI.

For validating dynamic velocity planning: The dvp_validation node acts as a UDP client to the UDP server running on the Autoware.AI side
For validating tracked objects publishing: The dvp_validation node binds to a port and listens for incoming objects packets.

# Using the validation node

## Validating dynamic velocity planning
A gamepad can be used with the validation node to test operation while operating Autoware.AI.
Assuming the same Logitech F310 gamepad that is used with the PacMod, the controls are as follows:

- "Start" Button: Enable velocity profile modification, using the last reported velocity of the vehicle for all waypoints.
- "Back" Button: Disable velocity profile modification, waypoints will revert to original velocities.
- "Directional Pad Up": Increase the velocity setpoint.
- "Directional Pad Down": Decrease the velocity setpoint.

## Validating tracked objects publishing
The dvp_validation node will listen for tracked object packets and publish RViz messages in order to visualize the received object data in RViz.
Subscribe to the `/tracked_objects_udp_viz` topic in RViz in order to see the data.
