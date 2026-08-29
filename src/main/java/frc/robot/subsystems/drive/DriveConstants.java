package frc.robot.subsystems.drive;

public final class DriveConstants {
  public static final int kLeftLeadMotorId = 1;
  public static final int kLeftFollowerMotorId = 7;
  public static final int kRightLeadMotorId = 3;
  public static final int kRightFollowerMotorId = 5;

  public static final boolean kLeftMotorInverted = false;
  public static final boolean kRightMotorInverted = true;

  public static final double kGearRatio = 6.122448979591837;

  public static final double kDriveKP = 0.1;
  public static final double kDriveKI = 0.0;
  public static final double kDriveKD = 0.0;
  public static final double kDriveKS = 0.0;
  public static final double kDriveKV = 0.124;

  public static final double kMaxVelocityRadPerSec = 100.0;
}
