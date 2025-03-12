# G-Force 2025 - Team 9028 Code & Hardware Specs

This repository contains the code for our 2025 season robot. We've created this documentation to help new team members understand our setup and to share our approach with other interested teams.

## Software Structure

Our robot runs on Java with WPILib as the foundation. The code is organized for efficient debugging during competition.

### How We've Organized Things

The main Robot.java file initializes everything and manages the robot's lifecycle. We've divided our code into logical subsystems (drivetrain, shooter, etc.) following WPILib's command-based architecture.

Commands handle specific robot actions and can be triggered by either driver input or autonomous routines. Our OI (Operator Interface) maps controller buttons to these commands, allowing drivers to focus on strategy rather than remembering complex control schemes.

For autonomous mode, we use PathPlanner to create smooth driving trajectories that maximize scoring potential during the 15-second autonomous period.

### Libraries We're Using

GradleRIO manages our dependencies, which include:
- WPILib for core functionality
- REV's library for NEO motor control
- CTRE Phoenix for CANcoder support
- PathPlanner for autonomous path generation

## Hardware Setup

We designed G-Force with a focus on balancing speed and precision control.

### Drivetrain

We've implemented a swerve drive with a custom aluminum frame. Each of the four modules contains two NEO motors - one for driving and one for steering. SparkMAX controllers manage the motors, while CTRE CANcoders provide absolute position feedback for precise module alignment.

### Game Piece Handling

Our intake uses a NEO motor with compliant wheels for secure game piece acquisition. The shooter features dual NEO motors with an adjustable hood for different shot trajectories. Limit switches ensure proper shooter positioning before firing.

### Sensors & Vision

Navigation relies on a NavX2 gyroscope for heading tracking. A Limelight 3 camera runs a custom pipeline for game piece detection. The built-in encoders in our NEO motors provide position feedback for all mechanisms.

### Control System

The system is controlled by a RoboRIO 2.0 with power managed through a REV PDH. Communications run through the Vivid-Hosting VH-109 radio for reliable field connectivity even in crowded competition environments.

## Contact Information

For questions or to share ideas, visit our team pit during competition or contact us through our team Discord.
