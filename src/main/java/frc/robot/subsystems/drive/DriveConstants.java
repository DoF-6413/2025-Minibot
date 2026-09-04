package frc.robot.subsystems.drive;

public final class DriveConstants {
  // Drive motor constants
  public static final int kLeftLeadMotorId = 2;
  public static final int kLeftFollowerMotorId = 8;
  public static final int kRightLeadMotorId = 4;
  public static final int kRightFollowerMotorId = 6;

  public static final boolean kLeftMotorInverted = false;
  public static final boolean kRightMotorInverted = true;

  public static final double kGearRatio = 6.122448979591837;

  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.124;

  public static final double kMaxVelocityRPS = 10;

  // Turn motor constants
  public static final double kTurnRatio = 8.53333333333 * 1.5;
  public static final double kEncoderFullRange = 2 * Math.PI;

  // Front left
  public static final int kFLTurnMotorId = 1;
  public static final int kFLEncoderPort = 0;
  public static final double kFLP = 1.0;
  public static final double kFLI = 0.0;
  public static final double kFLD = 0.0;
  public static final double kFLS = 0.0;
  public static final double kFLV = 0.124;
  public static final double kFLZero = 5.710; // 5.732

  // Front right
  public static final int kFRTurnMotorId = 3;
  public static final int kFREncoderPort = 1;
  public static final double kFRP = kFLP;
  public static final double kFRI = kFLI;
  public static final double kFRD = kFLD;
  public static final double kFRS = 0.0;
  public static final double kFRV = 0.124;
  public static final double kFRZero = 3.735; // 3.772

  // Back right
  public static final int kBRTurnMotorId = 5;
  public static final int kBREncoderPort = 2;
  public static final double kBRP = kFLP;
  public static final double kBRI = kFLI;
  public static final double kBRD = kFLD;
  public static final double kBRS = 0.0;
  public static final double kBRV = 0.124;
  public static final double kBRZero = 2.169; // 2.191

  // Back left
  public static final int kBLTurnMotorId = 7;
  public static final int kBLEncoderPort = 3;
  public static final double kBLP = kFLP;
  public static final double kBLI = kFLI;
  public static final double kBLD = kFLD;
  public static final double kBLS = 0.0;
  public static final double kBLV = 0.124;
  public static final double kBLZero = 5.8; // 5.789
}
