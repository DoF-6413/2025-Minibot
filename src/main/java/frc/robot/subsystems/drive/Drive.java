package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  public Drive(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    }
    holdWheelsStraight();
  }

  public void setSpeeds(double leftVolts, double rightVolts) {
    io.setLeftVoltage(leftVolts);
    io.setRightVoltage(rightVolts);
  }

  public void setVelocity(double leftRPS, double rightRPS) {
    io.setLeftVelocity(leftRPS);
    io.setRightVelocity(rightRPS);
  }

  public void holdWheelsStraight() {
    io.setTurnPositions(0.0, 0.0, 0.0, 0.0);
  }

  public void stop() {
    io.setLeftVoltage(0.0);
    io.setRightVoltage(0.0);
  }
}
