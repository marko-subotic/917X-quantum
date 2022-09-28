# 917Z Code Database
##### formerly 917X



## Repository Structure
   - PROS: Competition Code for 2021-22 VEX game tipping point
   - QuantumTests: Visual Studio Solution for writing tests for classes in competition code.
   - Excel Odom Data: Excel files with Data from sensors to analyze 3-encoder odometry

## Competition Code Structure
 - DriveTrainState is an object that is given input from sensors and maintains a state of a robot
 - DriveTrainController is an object class that uses a DriveTrainState object to control motors to fulfill functions such as driving or turning to a point
- DigitalDebug operates the V5 Brain to show a visual debug feed of either the position or vision
- Utils has various geometric functions used in DriveTrainState and DriveTrainController and testing
