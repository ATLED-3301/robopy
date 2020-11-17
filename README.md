# robot-kinematichs
A kinematichs library for robotichs application - in development

---
# importing
simply import:

+ kinematichs.serial.parametrization
+ kinematichs.serial.serial_chain
+ kinematichs.parallel.parallel

in your python file,you may need to intall some library(like numpy, scipy ...). You can use requirements.txt to install all the libraries.

## details
classes :

+ joint - use Dinavit Hartemberg convention for joint definition
+ serial chain - use joint to define a serial robot
+ parallel - use serial chain to cretae a parallel robot 

Details for the single classes are in the markdown of the classes.

this library solve inverse kinematichs for a single serial robot or for a signle parallel robot.

You can assign coordinates x y z yaw pitch roll and the program can solve inverse kinematichs using Levenberg-Marquart algorithm. When you try to move a serial you assign end effector position, then you move parallel you assign the position of the floating base of the robot.

You can also use numpy.nan to tell the program you have no interest to assign that coordinates (usefull for not fully attuated robot where you cannot assign all coordinates).

Some examples is in jupiter test file

