# raspberrypi_workspace
Repository for Discrete Adaptive Sliding Mode Controller for multicopter with a suspended payload.

**Overview:**
This workspace has six main components.
1. Controller

  
3. Trajectory generator

   
5. Rosserial
[Rosserial](http://wiki.ros.org/rosserial_python) is a package that makes sending and parsing information over a serial connection easy. In this case, an Arduino connected to the payload encoders running the payload state transmitter code is connected via USB to the Raspberry Pi running ROS and the bulk of the code. The advantage of Rosserial is that your payload code can act as a ROS node, publishing and subscribing to topics, and you don't need a separate node on the receiving device to parse information. To use the Rosserial package, install the package in your main workspace at the same level as this package and other packages (the "src" directory of the workspace). Then, in the Rosserial package, edit "serial_node.py" to specify the applicable port and baudrate for the serial connection. And that's all you need to do! As long as you include the serial_node in the launch file, you should see the data from the Arduino (or other device) published on the topic specified in the payload state transmitter code.

7.  Payload state transmitter

   
8.  Wavelet transform

   
10.  Accelerometer hammer mechanism
- 
**Hardware build**
**Software build**
**Setup and checks**

**How to run**




**Other useful information:**
- github for simulator
- github for Freyja original
- 

License and credits:


