// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class RunIntake extends Command {
  public Intake m_intake;
  public Pivot m_pivot;
  public Hopper m_hopper;

  public RunIntake(Intake m_intake2, Pivot pivot, Hopper hopper) {
    m_intake = m_intake2;
    m_pivot = pivot;
    m_hopper = hopper;

    addRequirements(m_intake2, pivot, hopper); // TODO: investigate?
  }

  @Override
  public void initialize() {
    m_pivot.deployPivot();
  }

  @Override
  public void execute() {
    m_hopper.setVoltage(3.0);
    m_intake.setVoltage(3.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.setVoltage(0.0);
    m_intake.setVoltage(0.0);
  }
}
