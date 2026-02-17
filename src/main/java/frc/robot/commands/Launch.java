// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.shooter.ShooterConstants.SPINUP_SEC;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.column.ColumnConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Launch extends Command {
  public Shooter m_shooter;
  public Column m_feeder;
  public Hopper m_hopper;

  public Launch(Shooter shooter, Column feeder, Hopper hopper) {
    m_shooter = shooter;
    m_feeder = feeder;
    m_hopper = hopper;

    addRequirements(shooter, feeder, hopper);
  }

  @Override
  public void initialize() {
    m_feeder.setVoltage(ColumnConstants.INTAKING_VOLTAGE);
    m_shooter.setVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void execute() {
    new WaitCommand(SPINUP_SEC);
    m_feeder.setVoltage(ColumnConstants.LAUNCHING_VOLTAGE);
    m_shooter.setVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    m_feeder.setVoltage(0.0);
    m_shooter.setVoltage(0.0);
  }

  // TODO: COMMANDS TO IMPLEMENT!

  // /** Intake and store in hopper */
  // public Command intake(Intake m_intake) {
  //   return Commands.run(() -> {m_intake.deployPivot();})
  //       .alongWith(() -> {m_superstructure.setShooterVoltage(0.0);
  //       });
  // }

  // /** Runs the launcher sequence. From hopper, slowly ramp feeder */
  // public static Command launch(Superstructure m_superstructure) {
  //   return Commands.run(() -> {
  //         m_superstructure.setFeederVoltage(FeederConstants.INTAKING_VOLTAGE);
  //         m_superstructure.setShooterVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
  //       })
  //       .withTimeout(SuperstructureConstants.SPINUP_SEC)
  //       .andThen(
  //           Commands.run(
  //               () -> {
  //                 m_superstructure.setFeederVoltage(FeederConstants.LAUNCHING_VOLTAGE);
  //                 m_superstructure.setShooterVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
  //               }))
  //       .finallyDo(
  //           () -> {
  //             m_superstructure.setFeederVoltage(0.0);
  //             m_superstructure.setShooterVoltage(0.0);
  //           });
  // }
}
