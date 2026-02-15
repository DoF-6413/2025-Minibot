package frc.robot.subsystems.feeder;

import edu.wpi.first.math.util.Units;

//TODO: UPDATE ALL
public class FeederConstants {
    /* CAN ID for Kraken */
    public static final int CAN_ID = 23;

    // Geometry for calculations
    /* Gear reduction of 50:1 */
    public static final double GEAR_RATIO = 50.0/1.0;

    /* Sets inversion of motor to false, making CCW = positive direction */
    public static final boolean IS_INVERTED = false;
    public static final boolean IS_BRAKE_MODE_ENABLED = false;

    /* Current limiting */
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    public static final int CURRENT_LIMIT = 30;

    /* Motor Configs */
    /* Refreshes TalonFX signals 50 times a second (every 0.02 seconds) */
    public static final double UPDATE_FREQUENCY_HZ = 50;

    /* Voltages */
    public static final double DEFAULT_VOLTAGE = 5.0;

    /* PID & FF Constants */ 
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
}
