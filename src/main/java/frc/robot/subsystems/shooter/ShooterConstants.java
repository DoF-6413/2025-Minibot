// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

// TODO: Verify all
public final class ShooterConstants {
  public static final double SPINUP_SEC = 1.0;

  public static final int CAN_ID = 20;
  public static final double GEAR_RATIO = 1.0;
  public static final int CURRENT_LIMIT = 30;

  public static final double INTAKING_VOLTAGE = 0.0;
  public static final double LAUNCHING_VOLTAGE =
      -6.5; // 8 hits the ceiling, this static voltage was good for 132"

  // PID & FF constants
  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double DEFAULT_RPM = 3500;
  public static double TOLERANCE_RPM = 300;

  public static double kS = 0.0;
  public static double kV = 0.0;
  public static double kA = 0.0;
}
