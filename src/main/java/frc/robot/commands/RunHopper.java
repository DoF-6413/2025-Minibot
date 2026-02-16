// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;

public class RunHopper extends Command {
  public Hopper m_hopper;

  public RunHopper(Hopper hopper) {
    m_hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_hopper.setVoltage(6.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.setVoltage(0.0);
  }
}
