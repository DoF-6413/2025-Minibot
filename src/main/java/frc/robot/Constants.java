// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static class RobotStateConstants {
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

    public static Mode getMode() {
      if (RobotBase.isReal()) {
        return Mode.REAL;
      } else if (RobotBase.isSimulation()) {
        return Mode.SIM;
      } else {
        return Mode.REPLAY;
      }
    }

    /**
     * @return Alliance from FMS
     */
    public static Optional<Alliance> getAlliance() {
      return DriverStation.getAlliance();
    }

    /* Motor Configs */
    /* Refreshes TalonFX signals 50 times a second (every 0.02 seconds) */
    public static final double UPDATE_FREQUENCY_HZ = 50;
    /* Times out PHX tuner config after 0.25 sec */
    public static final double PHX_CONFIG_TIMEOUT_SEC = 0.25;
    /* Times out CAN bus after 30 sec */
    public static final int CAN_CONFIG_TIMEOUT_SEC = 30;

    public static final double PERIODIC_LOOP_SEC = 0.02;
    public static final double MAX_VOLTAGE = 12;

    /** Weight of robot with bumpers and battery */
    public static final double ROBOT_WEIGHT_KG = Units.lbsToKilograms(0.0); // TODO: Update
  }

  public static class OperatorConstants {
    public static int DRIVE_CONTROLLER = 0;
    public static int AUX_CONTROLLER = 1;
  }
}
