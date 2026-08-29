package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class DriveConstants {
  public static final int kLeftLeadMotorId = 1;
  public static final int kLeftFollowerMotorId = 7;
  public static final int kRightLeadMotorId = 3;
  public static final int kRightFollowerMotorId = 5;

  public static final boolean kLeftMotorInverted = false;
  public static final boolean kRightMotorInverted = true;

  public static final double kGearRatio = 6.122448979591837;

  public static final LoggedNetworkNumber kLeftKP = new LoggedNetworkNumber("Drive/Left kP", 0.1);
  public static final LoggedNetworkNumber kLeftKI = new LoggedNetworkNumber("Drive/Left kI", 0.0);
  public static final LoggedNetworkNumber kLeftKD = new LoggedNetworkNumber("Drive/Left kD", 0.0);
  public static final LoggedNetworkNumber kLeftKS = new LoggedNetworkNumber("Drive/Left kS", 0.0);
  public static final LoggedNetworkNumber kLeftKV = new LoggedNetworkNumber("Drive/Left kV", 0.124);

  public static final LoggedNetworkNumber kRightKP = new LoggedNetworkNumber("Drive/Right kP", 0.1);
  public static final LoggedNetworkNumber kRightKI = new LoggedNetworkNumber("Drive/Right kI", 0.0);
  public static final LoggedNetworkNumber kRightKD = new LoggedNetworkNumber("Drive/Right kD", 0.0);
  public static final LoggedNetworkNumber kRightKS = new LoggedNetworkNumber("Drive/Right kS", 0.0);
  public static final LoggedNetworkNumber kRightKV = new LoggedNetworkNumber("Drive/Right kV", 0.124);

  public static final double kMaxVelocityRPS = 600;
}
