# dynamixel_hardware

The [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for any kind of [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) robots.

The `dynamixel_hardware` package is the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation for the multiple ROBOTIS Dynamixel servos.

It is hopefully compatible any configuration of ROBOTIS Dynamixel servos thanks to the `ros2_control`'s flexible architecture.

## ROBOTIS Hardware

### Configure Dynamixel motor parameters

Update the `usb_port`, `baud_rate`, and joint `id`'s.  The easiest way to find there values is to use the dynamixel wizard. https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/

If you don't have real hardware, set the `use_dummy` parameter to true and the desired position will be mirred to current position.

You can use the `offset` (in Rads) and `gear_ratio` parameters to adjust the joint state and the desired state for calibrating the motor to your robot. The formulas are as follows:

```command_position = (command.position  * gear_ratio) + offset```

```state_position = (position - offset) / gear_ratio```

```xml
<hardware>
  <plugin>dynamixel_hardware/DynamixelHardware</plugin>
  <param name="usb_port">/dev/ttyUSB0</param>
  <param name="baud_rate">1000000</param>
  <param name="use_dummy">false</param>
  <param name="enable_torque">true</param>
  <param name="control_mode">CurrentBasedPosition</param><!-- Velocity Position CurrentBasedPosition -->
</hardware>
<joint name="gripper_motor_joint">
  <param name="id">${motors_params['gripper']['id']}</param>
  <param name="offset">${motors_params['gripper']['offset']}</param>
  <param name="gear_ratio">-2.5</param>

  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <command_interface name="effort"/>

  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```
