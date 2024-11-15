# Tank Drive with CAN

### Description
This example shows how to set up tank drive with two SPARK MAX controllers and two brushless motors.

### Usage
Set the variables at the beginning of the example to match your setup.
- `leftDeviceID` - Left motor CAN device ID
- `rightDeviceID` - right motor CAN device ID

Control the left motor by moving the left joystick up or down.

Control the right motor by moving the right joystick up or down.

The example can be modified to support brushed motors by changing the MotorType values to MotorType.kBrushed when 
creating the SPARK MAX objects.