# robot-kinematichs
A kinematichs library for robotichs application - in development

---
# usage
simply import:

+ kinematichs.serial.parametrization
+ kinematichs.serial.serial_chain
+ kinematichs.parallel.parallel

in your python file,you may need to intall some library, then build your robotic system using joint class and HD convention, setting all parameters you need.

classes :

+ joint - use Dinavit Hartemberg convention for joint definition
+ serial chain - use joint to define a serial robot
+ parallel - use serial chain to cretae a parallel robot 

The example is in jupiter test file


