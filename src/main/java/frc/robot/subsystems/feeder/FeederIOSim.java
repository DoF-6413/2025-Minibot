// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotStateConstants;

public class FeederIOSim implements FeederIO {
  private DCMotorSim feederSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, FeederConstants.GEAR_RATIO),
          DCMotor.getCIM(1));

  private double feederAppliedVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    feederSim.setInputVoltage(feederAppliedVolts);
    feederSim.update(RobotStateConstants.PERIODIC_LOOP_SEC);

    inputs.feederRPM = feederSim.getAngularVelocityRPM();
    inputs.feederAppliedVolts = feederAppliedVolts;
    inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    feederAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
