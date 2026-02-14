// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim feederSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, GEAR_RATIO),
          DCMotor.getCIM(1));
  private DCMotorSim shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getCIM(1), 0.004, shooterMotorReduction),
          DCMotor.getCIM(1));

  private double feederAppliedVolts = 0.0;
  private double shooterAppliedVolts = 0.0;

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {
    feederSim.setInputVoltage(feederAppliedVolts);
    feederSim.update(0.02);

    shooterSim.setInputVoltage(shooterAppliedVolts);
    shooterSim.update(0.02);

    inputs.feederPositionRad = feederSim.getAngularPositionRad();
    inputs.feederVelocityRadPerSec = feederSim.getAngularVelocityRadPerSec();
    inputs.feederAppliedVolts = feederAppliedVolts;
    inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();

    inputs.leftShooterPositionRad = shooterSim.getAngularPositionRad();
    inputs.leftShooterVelocityRadPerSec = shooterSim.getAngularVelocityRadPerSec();
    inputs.leftShooterAppliedVolts = shooterAppliedVolts;
    inputs.leftShooterCurrentAmps = shooterSim.getCurrentDrawAmps();
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setShooterVoltage(double volts) {
    shooterAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
