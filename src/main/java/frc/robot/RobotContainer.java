// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Declares subsystems, button bindings, and the autonomous/characterization chooser. */
public class RobotContainer {
  private final Drive drive;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /**
   * Outreach/demo speed cap, as a fraction of the drivetrain's theoretical max speed. Kept
   * conservative (nothing above 70%) since this robot is driven around the public. Read once at the
   * start of autonomous/teleop, not every periodic cycle.
   */
  private final LoggedDashboardChooser<Double> speedLimitChooser =
      new LoggedDashboardChooser<>("Speed Limit");

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot: TalonFX drive + turn, AnalogEncoder absolute position
        // (no CANcoder), NavX gyro.
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(DriveConstants.kFrontLeft),
                new ModuleIOTalonFX(DriveConstants.kFrontRight),
                new ModuleIOTalonFX(DriveConstants.kBackLeft),
                new ModuleIOTalonFX(DriveConstants.kBackRight));
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    speedLimitChooser.addOption("30%", 0.3);
    speedLimitChooser.addOption("40%", 0.4);
    speedLimitChooser.addDefaultOption("50%", 0.5);
    speedLimitChooser.addOption("60%", 0.6);
    speedLimitChooser.addOption("70%", 0.7);

    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command: field-relative joystick drive.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Hold A: snap to 0 degrees while still translating with the left stick.
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Press X: lock wheels in an X pattern to resist being pushed.
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Press B: reset the gyro heading to 0 (keeps current field position).
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Applies the dashboard-selected speed limit to the drivetrain. Call once at the start of
   * autonomous/teleop so the limit can be changed between matches without redeploying code.
   */
  public void updateSpeedLimit() {
    drive.setSpeedLimitPercent(speedLimitChooser.get());
  }
}
