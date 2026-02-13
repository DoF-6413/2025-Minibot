// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Set the rollers to the values for launching. Spins up before feeding fuel. */
  public Command intake() {
    return run(() -> {
          io.setIntakeVoltage(1.0);
          io.setAgitatorVoltage(1.0);
        })
        .withTimeout(feedingSeconds)
        .andThen(
            run(
                () -> {
                  io.setFeederVoltage(feederVoltage);
                  io.setAgitatorVoltage(agitatorVoltage);
                }))
        .finallyDo(
            () -> {
              io.setFeederVoltage(0.0);
              io.setShooterVoltage(0.0);
        });
  }
}
