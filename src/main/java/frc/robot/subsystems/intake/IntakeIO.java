package frc.robot.subsystems.intake;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double intakePositionRad = 0.0;
    public double intakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double intakeRPM = intakeVelocityRadPerSec * 60 / (2 * Math.PI) * IntakeConstants.GEAR_RATIO;

    public double hopperPositionRad = 0.0;
    public double hopperVelocityRadPerSec = 0.0;
    public double hopperAppliedVolts = 0.0;
    public double hopperCurrentAmps = 0.0;
    public double hopperRPM = hopperVelocityRadPerSec * 60 / (2 * Math.PI) * HopperConstants.GEAR_RATIO;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(SuperstructureIOInputs inputs) {}

  /** Run the feeder at the specified voltage. */
  public default void setFeederVoltage(double volts) {}

  /** Run the feeder at the specified position. */
  public default void setFeederPosition(double position) {}

  /** Run the intake and launcher at the specified voltage. */
  public default void setShooterVoltage(double volts) {}
}
