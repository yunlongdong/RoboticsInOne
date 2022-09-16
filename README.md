# RoboticsInOne
Robotics In One (RIO) Studio

[Chinese README](./README_CH.md)
## Introduction
RIO is committed to providing a standard and complete tool chain and ecology for the robot community, and providing a graphical operation interface. RIO realizes 3D visualization of robot Link, Joint and Center of Mass (CoM) by opening URDF file, and realizes robot kinematics/dynamics code generation via URDF ->MDH ->Kinematics ->Jacobian ->Dynamics.

![](./docs/res/urdf_view3d.jpg)
## Features
1. URDF file visualization, including Link (adjustable transparency), Axis, CoM, etc
2. Joints Control
3. Invert Joint Z-Axis can be manually adjusted to achieve joint configuration consistent with the real robot
4. Modified D-H exporting
5. Kinematics: forward kinematics, Jacobian symbolic representation, and code generation
6. Dynamics: quality matrix M, symbolic representation of Inverse Dynamics and code generation
7. One click generation of verification code: randomly generate test samples, and compare the generated code with the numerical solution of pybullet for verification
## TODO
* Interface: symoro is currently used for rigid body dynamics derivation; The code verification currently uses pybullet. In the future, we hope to provide more interfaces and leave room for expansion. It is better to expose some interfaces to allow users to write plug-ins to connect to any rigid body dynamics library.
* Code generation: derive the minimum parameter set, generate C++ and Python code for system identification, randomly generate test samples and calculate results

## Noted
1. URDF Joints Axis should be [0, 0, 1]
2. Currently, only URDF code generation with 1 subtree is supported

## Communication
RoboticsInOne QQ Chat Group：179412348
