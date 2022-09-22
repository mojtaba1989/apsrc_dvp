# Validation Setup

Start the Autoware system:
```
roslaunch aw_platform platform.launch
```
in separate terminal:
```
roslaunch aw_platform autonomy.launch
```

Start the validation system:

On a separate connected computer, or the same computer as Autoware:
1. Plug in the game pad.
2. Start the validation node:
```
roslaunch dvp_validation dvp_validation_node.launch
```
See the README.md in the dvp_validation package for instructions on using the node with the gamepad.

# Validation Scenarios

## MABx requests velocity profile modifications, but Autoware is not ready.

**Expected result:**  
Localization packet returns with `data_valid` and `velocity_profile_enabled` field set to false/0.

## MABx sends velocity profile with status field set to 0.

**Expected result:**  
If already/previously modifying velocity profile:  
Test is aborted and all waypoint velocities are immediately returned to original values from global path csv file.

If not currently modifying velocity profile:  
Velocity profile message is ignored.

## MABx sends velocity profile with status field set to 1.

**Expected result:**  
The velocity profile of the global path is modified according to the data in the received UDP packet.

## MABx sends velocity profile packet with incorrect CRC.

**Expected result:**  
The packet is ignored, but a localization/status packet is still sent back.

## MABx requests velocity profile that would result in velocity through a stop sign.

**Expected result:**  
Autoware disobeys the velocity profile request and slows down for stop sign.


## MABx requests velocity profile that would result in collision with object in the path.

**Expected result:**  
Autoware disobeys the velocity profile request and slows down for the object.


## Autoware detects and tracks objects in its field-of-view

**Expected result:**  
Tracked object UDP packets are sent out containing tracked objects information.
