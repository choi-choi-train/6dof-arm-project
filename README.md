# 6DOF Robotic Arm 🦾
*Welcome! 👋 This is a personal robotics project that I have been working on since spring of 2025. I began this project with zero knowledge of CAD, electrical engineering, microcontrollers, or what an M3 screw was. In my desire to grow these skills I've been working consistently on a 6DOF desktop robotic arm. Version 1 was done purely theoretically with no actual printing or manufacturing, and Version 2 is currently being worked on with my newfound knowledge in 3D printing and robotics design. Enjoy!*

🔥**OBJECTIVES**🔥

1) Develop CAD skills and intuition in designing parts suitable for machining and 3D printing
2) Build a human-scale arm that can lift at least 1kg from its end-effector
3) Learn basic inverse kinematics and apply to a simulated 3D environment
4) Print/machine designs, build and program electronics for design, integrate magnetic encoder feedback

🔥**OUTCOMES**🔥

✏️ [ONSHAPE] CAD design of 6DOF arm (Version 2 in progress)
   - NEMA17 Stepper Motor driven
   - Humanoid dimensions
   - 3D-printed Cycloidal gearboxes, Capstan drives

🐍 [PYTHON] PyBullet project modeling V1 of arm
   - Manual control of each DOF
   - Recording joint positions, replaying in sequence
   - Calculating inverse kinematics of goal frames in CSV file, finding alternative possible solutions for each orientation
   - Optimized replaying of sequence of joint positions

🔌 [C++] Stepper motor PWM control algorithm for Teensy4.0 & TMC2209 stepper driver
   - Cubic-bezier acceleration/deceleration profile
   - Rotation by a certain angle, scaling speed & acceleration profiles to match
   
📽️ VIDEOS:
   - V1 Arm CAD Showcase: https://drive.google.com/file/d/17bpIJcrS7KBjyCcSQjnxdCV7GBqHDyYh/view?usp=sharing
   - V1 Arm PyBullet Model Showcase: https://drive.google.com/file/d/1LGYSVCzqaMIUf3A3PrS6AKm3KjFHaaDS/view?usp=sharing
   - V2 Arm Cycloidal Gearbox Testbed Operation: https://drive.google.com/file/d/1Z99XrYdSyfT1g2gOHz0_v86vHNOj1S-u/view?usp=sharing
   - V2 Arm Cycloidal Gearbox -> Capstan Drive Shoulder Joint Operation: https://drive.google.com/file/d/1UcA1aLKyCWc_PHmPhC-W2EXKkLFnNGdU/view?usp=sharing
