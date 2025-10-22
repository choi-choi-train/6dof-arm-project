# 6DOF Robotic Arm
**OBJECTIVES**
1) Develop CAD skills and intuition in designing parts suitable for machining and 3D printing
2) Build a human-scale arm that can lift at least 1kg from its end-effector
3) Learn basic inverse kinematics and apply to a simulated 3D environment
4) Print/machine designs, build and program electronics for design, integrate magnetic encoder feedback

**OUTCOMES**
1) [ONSHAPE] CAD design of 6DOF arm (Version 2 in progress)
   - NEMA17 Stepper Motor driven
   - Humanoid dimensions
   - 3D-printed Cycloidal gearboxes, Capstan drives

2) [PYTHON] PyBullet project modeling V1 of arm
   - Manual control of each DOF
   - Recording joint positions, replaying in sequence
   - Calculating inverse kinematics of goal frames in CSV file, finding alternative possible solutions for each orientation
   - Optimized replaying of sequence of joint positions

3) [C++] Stepper motor PWM control algorithm for Teensy4.0 & TMC2209 stepper driver
   - Cubic-bezier acceleration/deceleration profile
   - Rotation by a certain angle, scaling speed & acceleration profiles to match
   
5) MEDIA:
   - V1 Arm CAD Showcase: https://drive.google.com/file/d/17bpIJcrS7KBjyCcSQjnxdCV7GBqHDyYh/view?usp=sharing
   - V1 Arm PyBullet Model Showcase: https://drive.google.com/file/d/1LGYSVCzqaMIUf3A3PrS6AKm3KjFHaaDS/view?usp=sharing
