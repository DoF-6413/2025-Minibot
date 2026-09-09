package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonFX;

public class RobotContainer {
  private final Drive drive;
  // private final CommandXboxController controller = new CommandXboxController(0);
  private final GenericHID leftJoystick = new GenericHID(0);
  private final GenericHID rightJoystick = new GenericHID(1);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(new DriveIOTalonFX());
        break;

      case SIM:
        drive = new Drive(new DriveIOSim());
        break;

      default:
        drive = new Drive(new DriveIO() {});
        break;
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.tankDrive(
            drive, () -> -leftJoystick.getRawAxis(1), () -> -rightJoystick.getRawAxis(1)));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
