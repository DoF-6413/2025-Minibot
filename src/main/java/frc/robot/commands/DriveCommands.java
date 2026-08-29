package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.01;

  private DriveCommands() {}

  public static Command tankDrive(
      Drive drive,
      DoubleSupplier leftSupplier,
      DoubleSupplier rightSupplier) {
    return Commands.run(
        () -> {
          double left = MathUtil.applyDeadband(leftSupplier.getAsDouble(), DEADBAND);
          double right = MathUtil.applyDeadband(rightSupplier.getAsDouble(), DEADBAND);

          drive.setVelocity(
              left * DriveConstants.kMaxVelocityRPS,
              right * DriveConstants.kMaxVelocityRPS);
        },
        drive);
  }
}
