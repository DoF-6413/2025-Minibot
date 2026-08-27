// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // CAN bus name shared by all swerve devices
  public static final String kCANBusName = "rio";

  // Theoretical free speed (m/s) at 12 V applied output
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(0.5);

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  /** Configuration for a single swerve module. */
  public record ModuleConstants(
      int driveMotorId,
      int steerMotorId,
      TalonFXConfiguration driveMotorInitialConfigs,
      Slot0Configs driveMotorGains,
      Slot0Configs steerMotorGains,
      double driveMotorGearRatio,
      double steerMotorGearRatio,
      double slipCurrent,
      boolean driveMotorInverted,
      boolean steerMotorInverted,
      ClosedLoopOutputType driveMotorClosedLoopOutput,
      ClosedLoopOutputType steerMotorClosedLoopOutput,
      double wheelRadius,
      double locationX,
      double locationY,
      double driveInertia,
      double steerInertia) {}

  // Front Left
  public static final ModuleConstants FrontLeft =
      new ModuleConstants(
          1, // driveMotorId
          2, // steerMotorId
          new TalonFXConfiguration(), // driveMotorInitialConfigs
          new Slot0Configs().withKP(0.0).withKI(0).withKD(0).withKS(0).withKV(0.0), // driveGains
          new Slot0Configs()
              .withKP(100)
              .withKI(0)
              .withKD(0.5)
              .withKS(0.1)
              .withKV(2.66)
              .withKA(0), // steerGains
          6.122448979591837, // driveMotorGearRatio
          21.428571428571427, // steerMotorGearRatio
          120.0, // slipCurrent
          false, // driveMotorInverted
          true, // steerMotorInverted
          ClosedLoopOutputType.Voltage, // driveMotorClosedLoopOutput
          ClosedLoopOutputType.Voltage, // steerMotorClosedLoopOutput
          0.0508, // wheelRadius (2 inches in meters)
          0.3048, // locationX (12 inches in meters)
          0.3048, // locationY (12 inches in meters)
          0.01, // driveInertia
          0.01); // steerInertia

  // Front Right
  public static final ModuleConstants FrontRight =
      new ModuleConstants(
          3, // driveMotorId
          4, // steerMotorId
          new TalonFXConfiguration(), // driveMotorInitialConfigs
          new Slot0Configs().withKP(0.0).withKI(0).withKD(0).withKS(0).withKV(0.0), // driveGains
          new Slot0Configs()
              .withKP(100)
              .withKI(0)
              .withKD(0.5)
              .withKS(0.1)
              .withKV(2.66)
              .withKA(0), // steerGains
          6.122448979591837, // driveMotorGearRatio
          21.428571428571427, // steerMotorGearRatio
          120.0, // slipCurrent
          true, // driveMotorInverted (right side)
          true, // steerMotorInverted
          ClosedLoopOutputType.Voltage, // driveMotorClosedLoopOutput
          ClosedLoopOutputType.Voltage, // steerMotorClosedLoopOutput
          0.0508, // wheelRadius
          0.3048, // locationX
          -0.3048, // locationY
          0.01, // driveInertia
          0.01); // steerInertia

  // Back Left
  public static final ModuleConstants BackLeft =
      new ModuleConstants(
          7, // driveMotorId
          8, // steerMotorId
          new TalonFXConfiguration(), // driveMotorInitialConfigs
          new Slot0Configs().withKP(0.0).withKI(0).withKD(0).withKS(0).withKV(0.0), // driveGains
          new Slot0Configs()
              .withKP(100)
              .withKI(0)
              .withKD(0.5)
              .withKS(0.1)
              .withKV(2.66)
              .withKA(0), // steerGains
          6.122448979591837, // driveMotorGearRatio
          21.428571428571427, // steerMotorGearRatio
          120.0, // slipCurrent
          false, // driveMotorInverted
          true, // steerMotorInverted
          ClosedLoopOutputType.Voltage, // driveMotorClosedLoopOutput
          ClosedLoopOutputType.Voltage, // steerMotorClosedLoopOutput
          0.0508, // wheelRadius
          -0.3048, // locationX
          0.3048, // locationY
          0.01, // driveInertia
          0.01); // steerInertia

  // Back Right
  public static final ModuleConstants BackRight =
      new ModuleConstants(
          5, // driveMotorId
          6, // steerMotorId
          new TalonFXConfiguration(), // driveMotorInitialConfigs
          new Slot0Configs().withKP(0.0).withKI(0).withKD(0).withKS(0).withKV(0.0), // driveGains
          new Slot0Configs()
              .withKP(100)
              .withKI(0)
              .withKD(0.5)
              .withKS(0.1)
              .withKV(2.66)
              .withKA(0), // steerGains
          6.122448979591837, // driveMotorGearRatio
          21.428571428571427, // steerMotorGearRatio
          120.0, // slipCurrent
          true, // driveMotorInverted (right side)
          true, // steerMotorInverted
          ClosedLoopOutputType.Voltage, // driveMotorClosedLoopOutput
          ClosedLoopOutputType.Voltage, // steerMotorClosedLoopOutput
          0.0508, // wheelRadius
          -0.3048, // locationX
          -0.3048, // locationY
          0.01, // driveInertia
          0.01); // steerInertia

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
