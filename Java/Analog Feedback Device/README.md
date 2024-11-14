# Analog Feedback Device

### Description
This example shows how to use an analog sensor as the feedback device for the closed loop controller of a REV NEO Brushless Motor instead of the NEO's built-in hall sensor. For demonstration, the closed loop controller will be used to drive the motor to the target position setpoint.

### Usage
Set the variables at the beginning of the example to match your setup.
- `deviceID` - CAN device ID

PID Coefficients can be adjusted on SmartDashboard as well as the set rotations (position setpoint). 

The setpoint and process variable will be displayed on SmartDashboard.