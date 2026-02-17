package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederTempCelsius = 0.0;
    public double feederRPM = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /**
   * Sets idle mode of motor
   *
   * @param enable {@code}true{@code} to enable brake mode, {@code}false{@code} for coast.
   */
  public default void enableBrakeMode(boolean enable) {}

  /** Run the feeder at the specified voltage. */
  public default void setVoltage(double volts) {}
}
