# Assembly guide of the robot
This file aims to provide a step-by-step guide of how to build the robot.

## 1. Chassis & motors
The chosen one is a commonly used structure, particularly well-known for learner robotics projects. It consists on four wheels attached to one DC motor each in differential-drive structure, connected to a solid base structure divided in two layers.

<img src="Files/chassis.jpg" alt="Chassis" width="300">

Further information about the DC motors can be found in the [docs/hardware](docs/hardware/) folder.
The power requirement of each motor ranges from 3 to 6 V. The four motors are connected in parallel, so a 6 V supply is sufficient to power all of them simultaneously. This voltage is provided by four AA batteries connected in series.

It is necessary a device to control all the motors, which is called a driver. This device receive some inputs and sends a PWM signal to each motor. In this case, the device is the L298N motor driver. Its datasheet is attached in the [docs/hardware](docs/hardware/) folder.

## 2. Processor
As a processor, we will use a Raspberry Pi 4 Model B with 8GB RAM. It will run all the codes and also connect all the electronic components. You can find more information about the Raspberry Pi directly in its [webpage](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/).

Here is attached a diagram with all its pins which will be used for the connections.
