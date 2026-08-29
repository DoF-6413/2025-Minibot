package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DriveIOSim implements DriveIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim leftSim;
  private final DCMotorSim rightSim;
  private final PIDController leftController = new PIDController(0, 0, 0);
  private final PIDController rightController = new PIDController(0, 0, 0);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean leftClosedLoop = false;
  private boolean rightClosedLoop = false;
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;

  public DriveIOSim() {
    leftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, 0.01, DriveConstants.kGearRatio),
            GEARBOX);
    rightSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, 0.01, DriveConstants.kGearRatio),
            GEARBOX);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Update PID gains from tunable constants
    leftController.setP(DriveConstants.kLeftKP.get());
    leftController.setI(DriveConstants.kLeftKI.get());
    leftController.setD(DriveConstants.kLeftKD.get());

    rightController.setP(DriveConstants.kRightKP.get());
    rightController.setI(DriveConstants.kRightKI.get());
    rightController.setD(DriveConstants.kRightKD.get());

    if (leftClosedLoop) {
      leftAppliedVolts = leftFFVolts + leftController.calculate(leftSim.getAngularVelocityRadPerSec());
    }
    if (rightClosedLoop) {
      rightAppliedVolts = rightFFVolts + rightController.calculate(rightSim.getAngularVelocityRadPerSec());
    }

    leftSim.setInputVoltage(MathUtil.clamp(leftAppliedVolts, -12.0, 12.0));
    rightSim.setInputVoltage(MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
    leftSim.update(0.02);
    rightSim.update(0.02);

    inputs.leftConnected = true;
    inputs.leftPositionRad = leftSim.getAngularPositionRad();
    inputs.leftVelocityRadPerSec = leftSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());

    inputs.rightConnected = true;
    inputs.rightPositionRad = rightSim.getAngularPositionRad();
    inputs.rightVelocityRadPerSec = rightSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftClosedLoop = false;
    leftAppliedVolts = volts;
  }

  @Override
  public void setRightVoltage(double volts) {
    rightClosedLoop = false;
    rightAppliedVolts = volts;
  }

  @Override
  public void setLeftVelocity(double velocityRadPerSec) {
    leftClosedLoop = true;
    leftController.setSetpoint(velocityRadPerSec);
    leftFFVolts = DriveConstants.kLeftKS.get() * Math.signum(velocityRadPerSec) + DriveConstants.kLeftKV.get() * velocityRadPerSec;
  }

  @Override
  public void setRightVelocity(double velocityRadPerSec) {
    rightClosedLoop = true;
    rightController.setSetpoint(velocityRadPerSec);
    rightFFVolts = DriveConstants.kRightKS.get() * Math.signum(velocityRadPerSec) + DriveConstants.kRightKV.get() * velocityRadPerSec;
  }
}
