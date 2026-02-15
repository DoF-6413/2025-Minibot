// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperstructureConstants.FeederConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

  public Superstructure(SuperstructureIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure", inputs);
  }

  public void setFeederVoltage(double volts) {
    io.setFeederVoltage(volts);
  }

  public void setShooterVoltage(double volts) {
    io.setShooterVoltage(volts);
  }

  /** Runs the launcher sequence. From hopper, slowly ramp feeder */
  public static Command launch(Superstructure m_superstructure) {
    return Commands.run(
            () -> {
              m_superstructure.setFeederVoltage(FeederConstants.INTAKING_VOLTAGE);
              m_superstructure.setShooterVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
            })
        .withTimeout(SuperstructureConstants.SPINUP_SEC)
        .andThen(
            Commands.run(
                () -> {
                  m_superstructure.setFeederVoltage(FeederConstants.LAUNCHING_VOLTAGE);
                  m_superstructure.setShooterVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
                }))
        .finallyDo(
            () -> {
              m_superstructure.setFeederVoltage(0.0);
              m_superstructure.setShooterVoltage(0.0);
            });
  }
}
