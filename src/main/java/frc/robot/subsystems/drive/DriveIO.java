package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;

    public double FLAngleRot = 0.0;
    public double FRAngleRot = 0.0;
    public double BRAngleRot = 0.0;
    public double BLAngleRot = 0.0;
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setLeftVoltage(double volts) {}

  public default void setRightVoltage(double volts) {}

  public default void setLeftVelocity(double velocityRadPerSec) {}

  public default void setRightVelocity(double velocityRadPerSec) {}

  public default void setTurnPositions(double fl, double fr, double bl, double br) {}
}
