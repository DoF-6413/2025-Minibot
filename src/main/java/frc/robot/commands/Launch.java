// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants.FeederConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.ShooterConstants;

public class Launch extends Command {
  public Superstructure superstructure;

  public Launch(Superstructure superstructure) {
    this.superstructure = superstructure;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.setFeederVoltage(FeederConstants.INTAKING_VOLTAGE);
    superstructure.setShooterVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void execute() {
    superstructure.setFeederVoltage(FeederConstants.LAUNCHING_VOLTAGE);
    superstructure.setShooterVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setFeederVoltage(0.0);
    superstructure.setShooterVoltage(0.0);
  }

  // TODO: COMMANDS TO IMPLEMENT!

  // /** Intake and store in hopper */
  // public Command intake(Intake m_intake) {
  //   return Commands.run(() -> {m_intake.deployPivot();})
  //       .alongWith(() -> {m_superstructure.setShooterVoltage(0.0);
  //       });
  // }

  // /** Set the rollers to the values for ejecting fuel out the intake. */
  // public Command eject() {
  //   return runEnd(
  //       () -> {
  //         io.setFeederVoltage(-intakingFeederVoltage);
  //         io.setIntakeLauncherVoltage(-intakingFeederVoltage);
  //       },
  //       () -> {
  //         io.setFeederVoltage(0.0);
  //         io.setIntakeLauncherVoltage(0.0);
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
