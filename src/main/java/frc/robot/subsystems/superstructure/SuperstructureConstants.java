// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

public final class SuperstructureConstants {
  public static final class ShooterConstants{
    public static final int shooterCanId = 20; //TODO: verify with A-bot
    public static final double shooterMotorReduction = 1.0;
    public static final int shooterCurrentLimit = 30;
    
    public static final double intakingShooterVoltage = 10.0;
    public static final double launchingShooterVoltage =
    -6.5; // 8 hits the ceiling, this static voltage was good for 132"
    
    public static final double spinUpFeederVoltage = 6.0;
    public static final double spinUpSeconds = 1.0;
    
    // PID & FF constants
    public static double kP = 0.0; //TODO: test if different values for each motor are needed
    public static double kI = 0.0;
    public static double kD = 0.0;
    
    public static double setpointRPM; //TODO: update
    public static double toleranceRPM; //TODO: update
    
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
  }

  public static final class FeederConstants {
    public static final int CAN_ID = 21; //TODO: verify with A-bot
    public static final double GEAR_RATIO = 1.0;
    public static final int CURRENT_LIMIT = 30;
  
    public static final double INTAKING_VOLTAGE = -12.0; //TODO: verify
    public static final double LAUNCHING_VOLTAGE = 9.0; //TODO: verify
  }
}
