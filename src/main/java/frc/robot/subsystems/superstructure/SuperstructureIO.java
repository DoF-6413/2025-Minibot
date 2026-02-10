package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.intakeLauncherMotorReduction;

import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
  @AutoLog
  public static class SuperstructureIOInputs {
    public double feederPositionRad = 0.0;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederRPM =
        feederVelocityRadPerSec * 60 / (2 * Math.PI) * intakeLauncherMotorReduction;

    public double intakeLauncherPositionRad = 0.0;
    public double intakeLauncherVelocityRadPerSec = 0.0;
    public double intakeLauncherAppliedVolts = 0.0;
    public double intakeLauncherCurrentAmps = 0.0;
    public double intakeLauncherRPM =
        intakeLauncherVelocityRadPerSec * 60 / (2 * Math.PI) * intakeLauncherMotorReduction;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(SuperstructureIOInputs inputs) {}

  /** Run the feeder at the specified voltage. */
  public default void setFeederVoltage(double volts) {}

  /** Run the intake and launcher at the specified voltage. */
  public default void setIntakeLauncherVoltage(double volts) {}
}
