// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants.FeederConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.ShooterConstants;

public class Intake extends Command {
  public Intake m_intake;
  public Pivot m_pivot;
  public Hopper m_hopper;

  public Intake(Intake intake, Pivot pivot, Hopper hopper) {
    // addRequirements(intake, hopper); //TODO: investigate?
    
    m_intake = intake;
    m_pivot = pivot;
    m_hopper = hopper;
  }

  @Override
  public void initialize() {
    m_pivot.deployPivot();
  }

  @Override
  public void execute() {
    m_intake.setVoltage(3.0);
    m_hopper.setVoltage(3.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setVoltage(0.0);
    m_hopper.setVoltage(0.0);
  }
}
