// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import java.util.Queue;

/**
 * Module IO implementation for a TalonFX drive motor, TalonFX turn motor, and a WPILib {@link
 * AnalogEncoder} for absolute turn position — no CANcoder. The turn Falcon closes its position loop
 * on its own internal rotor sensor ({@code SensorToMechanismRatio} = turn gear ratio); at
 * construction its rotor position is seeded from the analog encoder's absolute reading, exactly as
 * this repo's TankDrive branch already does successfully.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final AnalogEncoder turnEncoder;
  private final AnalogInput turnEncoderAnalogInput;
  private final ModuleConstants constants;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  // Set when a Talon reset is detected but the analog encoder doesn't look
  // healthy yet, so the re-seed is deferred rather than trusting a possibly
  // rail-pinned reading. Retried every updateInputs() cycle until the
  // encoder reads healthy again.
  private boolean turnReseedPending = false;

  public ModuleIOTalonFX(ModuleConstants constants) {
    this.constants = constants;
    driveTalon = new TalonFX(constants.driveMotorId(), DriveConstants.kCANBus);
    turnTalon = new TalonFX(constants.turnMotorId(), DriveConstants.kCANBus);
    // AnalogEncoder(channel, fullRange, expectedZero): full range is 2*PI
    // radians, offset is the raw voltage-derived reading at the mechanical
    // zero position (measured on TankDrive).
    turnEncoder =
        new AnalogEncoder(
            constants.encoderChannel(), 2.0 * Math.PI, constants.encoderOffsetRadians());
    // Separate raw AnalogInput on the same channel, used only to read the raw
    // voltage for connection-health checking (AnalogEncoder itself exposes no
    // connection-health API).
    turnEncoderAnalogInput = new AnalogInput(constants.encoderChannel());

    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        constants.driveMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.kDriveGearRatio;
    driveConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kDriveP)
            .withKI(DriveConstants.kDriveI)
            .withKD(DriveConstants.kDriveD)
            .withKS(DriveConstants.kDriveS)
            .withKV(DriveConstants.kDriveV);
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.kDriveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted =
        DriveConstants.kTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.Feedback.SensorToMechanismRatio = DriveConstants.kTurnGearRatio;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kTurnP)
            .withKI(DriveConstants.kTurnI)
            .withKD(DriveConstants.kTurnD)
            .withKS(DriveConstants.kTurnS)
            .withKV(DriveConstants.kTurnV);
    turnConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.kTurnCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
    // Seed the Falcon's internal rotor position from the analog encoder's
    // absolute reading (Research.md step 3).
    seedTurnPosition();

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
  }

  /**
   * Seeds the turn Falcon's internal rotor position from the analog encoder's absolute reading.
   * Called once at construction, and again from {@link #updateInputs} any time a Talon reset is
   * detected, since a reset (e.g. brownout) discards the previously seeded rotor position.
   */
  private void seedTurnPosition() {
    tryUntilOk(5, () -> turnTalon.setPosition(Units.radiansToRotations(turnEncoder.get()), 0.25));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);

    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    // A disconnected/unpowered analog encoder typically reads a voltage
    // pinned near a rail (near 0V or near the 5V supply) rather than a
    // normal in-range reading, so use raw voltage as the health signal.
    double turnEncoderVoltage = turnEncoderAnalogInput.getVoltage();
    inputs.turnEncoderConnected =
        turnEncoderConnectedDebounce.calculate(
            turnEncoderVoltage > 0.1 && turnEncoderVoltage < 4.9);

    // Re-seed the turn Falcon's rotor position after a reset (e.g. brownout),
    // since the reset discards the previously seeded position. Only trust the
    // analog encoder's reading once it looks healthy again: a reset can
    // coincide with a broader brownout that also disturbs the analog rail, so
    // seeding from a momentarily bad reading would silently point the module
    // at the wrong angle with no warning. If the encoder isn't healthy yet,
    // defer and retry every cycle until inputs.turnEncoderConnected is true.
    if (turnTalon.hasResetOccurred()) {
      turnReseedPending = true;
    }
    if (turnReseedPending) {
      if (inputs.turnEncoderConnected) {
        DriverStation.reportWarning(
            "Turn Falcon (CAN ID "
                + constants.turnMotorId()
                + ", drive CAN ID "
                + constants.driveMotorId()
                + ") reset detected; re-seeding turn position from analog encoder.",
            false);
        seedTurnPosition();
        turnReseedPending = false;
      } else {
        DriverStation.reportWarning(
            "Turn Falcon (CAN ID "
                + constants.turnMotorId()
                + ", drive CAN ID "
                + constants.driveMotorId()
                + ") reset detected, but analog encoder reads unhealthy (voltage "
                + turnEncoderVoltage
                + "V); deferring re-seed until it recovers.",
            false);
      }
    }

    inputs.turnAbsolutePosition = new Rotation2d(turnEncoder.get());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double v) -> v).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double v) -> Units.rotationsToRadians(v))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double v) -> Rotation2d.fromRotations(v))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveTalon.setControl(
        velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionRequest.withPosition(rotation.getRotations()));
  }
}
