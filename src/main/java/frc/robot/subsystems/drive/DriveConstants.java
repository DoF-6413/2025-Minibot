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
  public static final double kTurnRatio = 21.428571428571427;

  // Front left
  public static final int kFLTurnMotorId = 1;
  public static final int kFLEncoderPort = 0;
  public static final double kFLP = 0.1;
  public static final double kFLI = 0.0;
  public static final double kFLD = 0.0;
  public static final double kFLS = 0.0;
  public static final double kFLV = 0.124;
  public static final double kFLZero = 0.904;

  // Front right
  public static final int kFRTurnMotorId = 3;
  public static final int kFREncoderPort = 1;
  public static final double kFRP = 0.1;
  public static final double kFRI = 0.0;
  public static final double kFRD = 0.0;
  public static final double kFRS = 0.0;
  public static final double kFRV = 0.124;
  public static final double kFRZero = 0.592;

  // Back right
  public static final int kBRTurnMotorId = 5;
  public static final int kBREncoderPort = 2;
  public static final double kBRP = 0.1;
  public static final double kBRI = 0.0;
  public static final double kBRD = 0.0;
  public static final double kBRS = 0.0;
  public static final double kBRV = 0.124;
  public static final double kBRZero = 0.344;

  // Back left
  public static final int kBLTurnMotorId = 7;
  public static final int kBLEncoderPort = 3;
  public static final double kBLP = 0.1;
  public static final double kBLI = 0.0;
  public static final double kBLD = 0.0;
  public static final double kBLS = 0.0;
  public static final double kBLV = 0.124;
  public static final double kBLZero = 0.923;
}
