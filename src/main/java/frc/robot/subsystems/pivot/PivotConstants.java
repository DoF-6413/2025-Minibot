package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;

// TODO: UPDATE ALL
public class PivotConstants {
  /* CAN ID for Kraken */
  public static final int CAN_ID = 24;

  // Geometry for calculations
  /* Gear reduction of 50:1 */
  public static final double GEAR_RATIO = 50.0 / 1.0;
  /* Length of Pivot in meters */
  public static final double LENGTH_M = Units.inchesToMeters(0.0);
  /* Weight of Pivot in kilograms */
  public static final double WEIGHT_KG = Units.lbsToKilograms(0.0);

  /* Sets inversion of motor to false, making CCW = positive direction */
  public static final boolean IS_INVERTED = false;
  public static final boolean IS_BRAKE_MODE_ENABLED = false;

  /* Current limiting */
  public static final boolean ENABLE_CURRENT_LIMIT = true;
  public static final int CURRENT_LIMIT = 10;

  /* Angle positions */
  public static final double MIN_ANGLE_RAD = Units.degreesToRadians(0.0); // TODO: measure
  public static final double MAX_ANGLE_RAD = Units.degreesToRadians(0.0); // TODO: measure

  public static final double STOW_ANGLE_RAD = MIN_ANGLE_RAD + Units.degreesToRadians(5);
  public static final double DEPLOY_ANGLE_RAD = Units.degreesToRadians(0.0); // TODO: measure

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
