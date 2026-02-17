package frc.robot.subsystems.hopper;

// TODO: UPDATE ALL
public class HopperConstants {
  /* CAN ID for Kraken */
  public static final int CAN_ID = 23;

  // Geometry for calculations
  /* Gear reduction of 1:1 */
  public static final double GEAR_RATIO = 1.0 / 1.0;

  /* Sets inversion of motor to false, making CCW = positive direction */
  public static final boolean IS_INVERTED = false;
  public static final boolean IS_BRAKE_MODE_ENABLED = false;

  /* Current limiting */
  public static final boolean ENABLE_CURRENT_LIMIT = true;
  public static final int CURRENT_LIMIT = 30;

  public static final double LAUNCHING_VOLTAGE = -6.0;

  /* PID & FF Constants */
  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double TOLERANCE_RAD = 0.0;

  public static double kS = 0.0;
  public static double kV = 0.0;
  public static double kA = 0.0;

  public static double MAX_VELOCITY_DEG_PER_S = 0.0;
  public static double MAX_ACCELERATION_DEG_PER_S2 = 0.0;

  public static double MOI_KG_M2 = 0.0;
}
