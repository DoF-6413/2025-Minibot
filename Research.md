Q: Can the AdvantageKit swerve template work with a robot that uses Falcon 500 motors and Analog encoders?


A: Yes, the template will still work, but you cannot use it completely out-of-the-box. Because AdvantageKit isolates your code logic from the physical components using an Input/Output (IO) layer, you can adapt it to any hardware by swapping out the sensor implementation.

Refs:
https://www.chiefdelphi.com/t/swerve-offsets/449687/2
https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/

To make your analog encoders (like the Thrifty Absolute Magnetic Encoder or similar options) work, you must modify the SwerveModuleIO layer:

1. Update the IO InterfaceOpen your SwerveModuleIO.java file. Ensure that the ModuleIOInputs class includes a field to log the absolute position of your turn encoder. The template typically has this by default to support CANcoders.

2. Rewrite the Real Hardware ImplementationOpen SwerveModuleIOTalonFX.java. This is where the code communicates with physical components.

 - Remove the CANcoder objects and their associated setup logic.
 - Instantiate WPILib's AnalogInput or AnalogEncoder class for each of your four modules.
 - In the updateInputs method, read the position using .getAbsolutePosition() or .getVoltage() from your analog objects, and assign those values directly to the inputs data structure.

3. Handle Seeding/Zeroing

The template uses the absolute encoder to seed the Falcon 500's internal rotor encoder at startup. Since you are reading from an analog port instead of CAN, convert your analog readings into radians or rotations before pushing the position target to the Falcon 500 steering motor.

Refs:
https://www.chiefdelphi.com/t/swerve-encoders/437669

Would you like help writing the Java code snippet for the modified SwerveModuleIOTalonFX class, or do you need assistance determining how to wire the encoders to the roboRIO Analog Input ports?


Prompt to restart with:
We are configuring the Littleton Robotics AdvantageKit TalonFX Swerve Template for an FRC robot. The robot uses Falcon 500 motors for drive and steering, but uses Analog Encoders (instead of CANcoders) for absolute module positioning. We discussed adapting the template by modifying the SwerveModuleIO interface and rewriting SwerveModuleIOTalonFX.java to read from WPILib AnalogInput/AnalogEncoder classes to seed the Falcon steering motor positions.

Please acknowledge this setup. To resume, let's start by modifying the stock AdvantageKit Swerve drive subsystem template Java code.