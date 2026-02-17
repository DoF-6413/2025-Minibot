// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO m_io;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    System.out.println("[INIT] Feeder");
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Feeder", m_inputs);
  }

  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }
}
