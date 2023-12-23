# Robot Kinematics Library

A kinematics library for robotic applications - currently in development.

## Importing

To use this library, simply import the following modules in your Python file:

import kinematics.serial.parametrization
import kinematics.serial.serial_chain
import kinematics.parallel.parallel

Make sure to install the required libraries (e.g., numpy, scipy) by using the provided requirements.txt file.

## Details

### Classes
- Joint

    The Joint class utilizes the Denavit-Hartenberg convention for joint           definition.

- Serial Chain

    The Serial Chain class uses the Joint class to define a serial robot.

- Parallel

    The Parallel class uses the Serial Chain class to create a parallel robot.

For detailed information about each class, refer to the individual markdown files for the respective classes.

## Functionality

This library solves inverse kinematics for both a single serial robot and a single parallel robot.

You can assign coordinates (x, y, z, yaw, pitch, roll), and the program can solve inverse kinematics using the Levenberg-Marquart algorithm. When moving a serial robot, assign the end effector position; when moving a parallel robot, assign the position of the floating base of the robot.

Additionally, you can use numpy.nan to indicate to the program that you have no interest in assigning specific coordinates. This is particularly useful for not fully actuated robots where not all coordinates can be assigned.

## Examples

Check out the Jupyter test file for some usage examples.

## License

This project is licensed under the MIT License. Feel free to contribute and improve this library!

