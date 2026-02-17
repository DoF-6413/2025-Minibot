// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;

public class Feed extends Command {
  public Hopper m_hopper;
  public Feeder m_feeder;

  public Feed(Hopper hopper, Feeder feeder) {
    m_hopper = hopper;
    m_feeder = feeder;
    addRequirements(hopper, feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_hopper.setVoltage(HopperConstants.LAUNCHING_VOLTAGE);
    m_feeder.setVoltage(FeederConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.setVoltage(0.0);
    m_feeder.setVoltage(0);
  }
}
