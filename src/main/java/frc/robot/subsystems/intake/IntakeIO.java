package frc.robot.subsystems.intake;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class SuperstructureIOInputs {
    public double feederPositionRad = 0.0;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederRPM = feederVelocityRadPerSec * 60 / (2 * Math.PI) * GEAR_RATIO;

    public double leftShooterPositionRad = 0.0;
    public double leftShooterVelocityRadPerSec = 0.0;
    public double leftShooterAppliedVolts = 0.0;
    public double leftShooterCurrentAmps = 0.0;
    public double leftShooterRPM = leftShooterVelocityRadPerSec * 60 / (2 * Math.PI) * shooterMotorReduction;

    public double midShooterPositionRad = 0.0;
    public double midShooterVelocityRadPerSec = 0.0;
    public double midShooterAppliedVolts = 0.0;
    public double midShooterCurrentAmps = 0.0;
    public double midShooterRPM = midShooterVelocityRadPerSec * 60 / (2 * Math.PI) * shooterMotorReduction;
    
    public double rightShooterPositionRad = 0.0;
    public double rightShooterVelocityRadPerSec = 0.0;
    public double rightShooterAppliedVolts = 0.0;
    public double rightShooterCurrentAmps = 0.0;
    public double rightShooterRPM = rightShooterVelocityRadPerSec * 60 / (2 * Math.PI) * shooterMotorReduction;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(SuperstructureIOInputs inputs) {}

  /** Run the feeder at the specified voltage. */
  public default void setFeederVoltage(double volts) {}

  /** Run the intake and launcher at the specified voltage. */
  public default void setShooterVoltage(double volts) {}
}
