package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Real-robot constants for the swerve drivetrain: 4x Falcon 500 drive, 4x Falcon 500 turn, 4x
 * WPILib AnalogEncoder absolute turn encoders (no CANcoders), NavX gyro. CAN IDs, gear ratios,
 * encoder ports/offsets, and PID gains below are measured values carried over verbatim from the
 * working {@code TankDrive} branch. Chassis geometry and max speed are flagged where they are still
 * placeholders pending exact on-robot measurement.
 */
public final class DriveConstants {
  private DriveConstants() {}

  /** All swerve devices are on the roboRIO CAN bus (no CANivore). */
  public static final CANBus kCANBus = new CANBus();

  // Wheel / gearing (SDS MK3, L3-equivalent gearing; confirmed by the team and
  // matches the working TankDrive branch).
  public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final double kDriveGearRatio = 6.122448979591837;
  public static final double kTurnGearRatio = 12.8;

  // Chassis geometry.
  // TODO: verify on robot - measured as "about 16x16 with wheels about 11-12
  // inches center to center" per team estimate on 2026-09-08. Replace with
  // exact measurements before competition use.
  private static final double kTrackWidthMeters = Units.inchesToMeters(11.5);
  private static final double kWheelBaseMeters = Units.inchesToMeters(11.5);
  private static final double kHalfTrackWidthMeters = kTrackWidthMeters / 2.0;
  private static final double kHalfWheelBaseMeters = kWheelBaseMeters / 2.0;

  // Theoretical max linear speed, derived from Falcon 500 free speed and the
  // drive gear ratio — not empirically measured/calibrated.
  // TODO: verify on robot.
  public static final double kMaxLinearSpeedMetersPerSec =
      DCMotor.getFalcon500(1).freeSpeedRadPerSec / kDriveGearRatio * kWheelRadiusMeters;

  /** Drive motor stator current limit (amps). Safe default; retune after testing. */
  public static final double kDriveCurrentLimitAmps = 60.0;

  // Drive closed-loop gains (measured, from TankDrive).
  public static final double kDriveP = 0.1;
  public static final double kDriveI = 0.0;
  public static final double kDriveD = 0.0;
  public static final double kDriveS = 0.0;
  public static final double kDriveV = 0.124;

  // Turn closed-loop gains (measured, from TankDrive).
  public static final double kTurnP = 60.0;
  public static final double kTurnI = 0.0;
  public static final double kTurnD = 0.0;
  public static final double kTurnS = 0.0;
  public static final double kTurnV = 0.124;

  /** All four turn motors are wired Clockwise_Positive (measured, from TankDrive). */
  public static final boolean kTurnMotorInverted = true;

  // Drive motor inversion follows the left/right convention measured on
  // TankDrive (left side normal, right side inverted).
  // TODO: verify on robot - TankDrive only ever ran these as CAN-follower
  // pairs, never as 4 independent motors; confirm no module fights itself.
  private static final boolean kLeftDriveInverted = false;
  private static final boolean kRightDriveInverted = true;

  /** Per-corner constants: CAN IDs, encoder port/offset, inversion, and kinematic position. */
  public record ModuleConstants(
      int driveMotorId,
      int turnMotorId,
      boolean driveMotorInverted,
      int encoderChannel,
      double encoderOffsetRadians,
      double xPosMeters,
      double yPosMeters) {}

  public static final ModuleConstants kFrontLeft =
      new ModuleConstants(
          2, 1, kLeftDriveInverted, 0, 5.710, kHalfWheelBaseMeters, kHalfTrackWidthMeters);
  public static final ModuleConstants kFrontRight =
      new ModuleConstants(
          4, 3, kRightDriveInverted, 1, 3.735, kHalfWheelBaseMeters, -kHalfTrackWidthMeters);
  public static final ModuleConstants kBackLeft =
      new ModuleConstants(
          8, 7, kLeftDriveInverted, 3, 5.8, -kHalfWheelBaseMeters, kHalfTrackWidthMeters);
  public static final ModuleConstants kBackRight =
      new ModuleConstants(
          6, 5, kRightDriveInverted, 2, 2.169, -kHalfWheelBaseMeters, -kHalfTrackWidthMeters);
}
