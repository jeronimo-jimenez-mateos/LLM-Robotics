# Does an LLM work as a controller for a mobile robot?
The aim of this repository is to create a mobile robot and connect it to a Large Language Model which will act as the robot controller. The robot will have the role of object finder, so its scope will be finding the desired object in a space.

In the image above you can find a block diagram of the structure of the project.
It mainly consists on two parts:
- A mobile robot equipped with a camera that takes photos of the environment.
- A PC that executes an LLM. This LLM receives the images and decides what action to take.

Both devices are connected via WebSocket.

## Structure of the repo
- **src** Codes and requirements
- **docs** Documentation about libraries, hardware...
- **assembly-guide.md** Step-by-step guide of the robot building process

## What do I need to recreate this project?
In order to run exactly this project, you should have:
- A Raspberry Pi 4 Model B
- A PC apart from the Raspberry to execute the model
- All the sensors and components specified [here](assembly-tutorial.md).
