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

### Direct Kinematics

Direct kinematics is the study of how the joint parameters of a robot affect its end-effector position and orientation in the workspace. It involves determining the position and orientation of the robot's end effector based on the joint angles or joint parameters. The mathematical representation of the robot's geometry and the transformation matrices between consecutive joints are used to compute the position and orientation of the end effector. In other words, direct kinematics answers the question: "Given the joint values, where is the end effector located and oriented in space?"

The direct kinematics equations are typically represented using homogeneous transformation matrices and can be expressed as a sequence of transformations from the base to the end effector. The result is a transformation matrix that describes the pose of the end effector with respect to the robot's base frame.

### Inverse Kinematics

Inverse kinematics, on the other hand, is the process of determining the joint parameters (angles or lengths) of a robot that will result in a specific desired position and orientation of the end effector. Inverse kinematics is essential for planning and controlling robot movements in tasks such as reaching a specific point in space or following a trajectory. Unlike direct kinematics, which goes from joint parameters to end effector pose, inverse kinematics goes from the desired end effector pose to the required joint parameters.

Inverse kinematics problems can be challenging and may not always have a unique solution. Iterative numerical methods, such as the Levenberg-Marquardt algorithm, are often employed to find solutions that satisfy the desired end effector pose while respecting the constraints of the robot.


## Examples

Check out the Jupyter test file for some usage examples.

## License

This project is licensed under the MIT License. Feel free to contribute and improve this library!

