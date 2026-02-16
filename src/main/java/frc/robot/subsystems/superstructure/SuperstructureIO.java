package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
  @AutoLog
  public static class SuperstructureIOInputs {
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederTempCelsius = 0.0;
    public double feederRPM = 0.0;

    public double shooterAppliedVolts = 0.0;
    public double shooterCurrentAmps = 0.0;
    public double shooterTempCelsius = 0.0;
    public double shooterRPM = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(SuperstructureIOInputs inputs) {}

  /**
   * Sets idle mode of motor
   *
   * @param enable {@code}true{@code} to enable brake mode, {@code}false{@code} for coast.
   */
  public default void setBrakeMode(boolean enable) {}

  /** Run the feeder at the specified voltage. */
  public default void setFeederVoltage(double volts) {}

  /** Run the intake and launcher at the specified voltage. */
  public default void setShooterVoltage(double volts) {}
}
