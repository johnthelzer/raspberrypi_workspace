# raspberrypi_workspace
Repository for Discrete Adaptive Sliding Mode Controller for multicopter with a suspended payload.

**Overview:**
This workspace has four main components. 
1. Controller

  
2. Trajectory generator
The trajectory generator (control_system->src->trajectory_generator)


3. Payload state transmitter
The payload state transmitter code (hosted here) runs on an external device (I used an Arduino Mega) mounted on the drone and reads the payload state. It publishes information to ROS using Rosserial (see below). In essence, the payload state transmitter code acts as a ROS node and publishes payload state data to a ROS topic on the base computer (Raspberry Pi). The way it's currently written, the payload state transmitter reads encoder angles, filters them with a low-pass filter, and publishes them. The important parameters in this node are the message type and contents and the topic the message should be published on.
   
4. Rosserial
[Rosserial](http://wiki.ros.org/rosserial_python) is a package that makes sending and parsing information over a serial connection easy. In this case, an Arduino connected to the payload encoders running the payload state transmitter code is connected via USB to the Raspberry Pi running ROS and the bulk of the code. To use the Rosserial package, install the package in your main workspace at the same level as this package and other packages (the "src" directory of the workspace). Then, in the Rosserial package, edit "serial_node.py" to specify the applicable port and baudrate for the serial connection. And that's all you need to do! As long as you include the serial_node in the launch file, you should see the data from the Arduino (or other device) published on the topic specified in the payload state transmitter code.


**Hardware build**
**Software build**
**Setup and checks**

**How to run**




**Other useful information:**
- github for simulator
- github for Freyja original
- 

License and credits:


