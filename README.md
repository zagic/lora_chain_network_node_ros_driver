# ROS1 driver for Lora chain network end-device

This project contains the ROS driver of the Lora chain network end-device.

The end-device will be controlled by the "driver_lora_chain_network_node"

It also comes with a sample node "lora_test_node" that allows the user to test the end-devcie and ineract with the end-devcie manually.

## Methodology
The driver interact with the end-device via serial port, so you need to connect the end-device to the ROS host via a USB2TTL dongle. 

The driver will create a service and other ROS nodes can send command to the end-device via the service. It will also create a topic and post any received message to that topic. 

The USB port name, servcie name and topic name are configurable. (refer to: lora_chain_network_driver.launch)

## Test the driver

You may need to install ROS serial driver first:

'$sudo apt-get install ros-noetic-serial'

Only ROS1 noetic was varified. However, it may also work with melodic or kinetic.

To test the driver, you need to build the project with 

'$catkin_make'

and run

'$source devel/setup.bash'

Then, you can start the ROS core and the driver_lora_chain_network_node

'$rosrun driver_lora_chain_network driver_lora_chain_network_node'

The test node must not be started in .launch file so that the manual input can work.

'$rosrun driver_lora_chain_network lora_test_node'

## More information

This driver only support unblock mode command of the end-devcie. Please refer to Lora chain network end device implementation_V_X_X.docx for more information about the end-device's behaviour.

For any Lora end-devcie with firmware V1_2_0 or later. The device would connect to repeater automatically. The user does not need to command the devcie to connect a repeater.
