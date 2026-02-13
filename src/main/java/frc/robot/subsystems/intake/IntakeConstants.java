// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int agitatorCanId = 23; //TODO: verify with A-bot
  public static final double agitatorMotorReduction = 1.0;
  public static final int agitatorCurrentLimit = 30;
  
  // public static final int pivotCanId = 24; //TODO: verify with A-bot
  // public static final double pivotMotorReduction = 1.0;
  // public static final int pivotCurrentLimit = 30;

  public static final int intakeCanId = 25; //TODO: verify with A-bot
  public static final double intakeMotorReduction = 1.0;
  public static final int intakeCurrentLimit = 30;

  public static final double agitatorVoltage = 6.0;
  // public static final double pivotVoltage = 6.0;
  public static final double intakeVoltage = 3.0;
  public static final double feedingSeconds = 1.0;

  // public static final double pivotIntakeVoltage = 3.0;
  // public static final double pivotAgitatorVoltage = 6.0;

  public static final double feedIntakeVoltage = 3.0;
  public static final double feedingAgitatorVoltage = 6.0;

  // PID & FF constants
  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double setpointRPM; //TODO: update
  public static double toleranceRPM; //TODO: update

  public static double kS = 0.0;
  public static double kV = 0.0;
  public static double kA = 0.0;
}
