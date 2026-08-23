// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class AnalogEncoderConstants {
    public static final double fullRangeRad = 2 * Math.PI;
    // Front left (Module 1)
    public static final int FLChannel = 0;
    public static final double FLZero = 0.0;
    // Front right (Module 2)
    public static final int FRChannel = 1;
    public static final double FRZero = 0.0;
    // Back right (Module 3)
    public static final int BRChannel = 2;
    public static final double BRZero = 0.0;
    // Back left (Module 4)
    public static final int BLChannel = 3;
    public static final double BLZero = 0.0;
  }
}
