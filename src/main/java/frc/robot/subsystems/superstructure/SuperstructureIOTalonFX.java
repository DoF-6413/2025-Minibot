// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;

/**
 * This superstructure implementation is for Talon FXs driving motors like the Falon 500, Kraken
 * X44, or Kraken X60.
 */
public class SuperstructureIOTalonFX implements SuperstructureIO {
  private final TalonFX feeder = new TalonFX(FeederConstants.CAN_ID);
  private final StatusSignal<AngularVelocity> feederVelocityRotPerSec = feeder.getVelocity();
  private final StatusSignal<Voltage> feederAppliedVolts = feeder.getMotorVoltage();
  private final StatusSignal<Current> feederCurrentAmps = feeder.getSupplyCurrent();
  private final StatusSignal<Temperature> feederTempCelsius = feeder.getDeviceTemp();

  private final TalonFX shooter = new TalonFX(ShooterConstants.CAN_ID);
  private final StatusSignal<AngularVelocity> shooterVelocityRotPerSec = shooter.getVelocity();
  private final StatusSignal<Voltage> shooterAppliedVolts = shooter.getMotorVoltage();
  private final StatusSignal<Current> shooterCurrentAmps = shooter.getSupplyCurrent();
  private final StatusSignal<Temperature> shooterTempCelsius = shooter.getDeviceTemp();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public SuperstructureIOTalonFX() {
    var feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.CURRENT_LIMIT;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> feeder.getConfigurator().apply(feederConfig, 0.25));

    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> shooter.getConfigurator().apply(shooterConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        feederVelocityRotPerSec,
        feederAppliedVolts,
        feederCurrentAmps,
        feederTempCelsius,
        shooterVelocityRotPerSec,
        shooterAppliedVolts,
        shooterCurrentAmps,
        shooterTempCelsius);
    ParentDevice.optimizeBusUtilizationForAll(feeder, shooter);
  }

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        feederVelocityRotPerSec,
        feederAppliedVolts,
        feederCurrentAmps,
        feederTempCelsius,
        shooterVelocityRotPerSec,
        shooterAppliedVolts,
        shooterCurrentAmps,
        shooterTempCelsius);

    // Motor rotations -> feeder rotations * 60 sec/min
    inputs.feederRPM = feederVelocityRotPerSec.getValueAsDouble() * 60 / FeederConstants.GEAR_RATIO;
    inputs.feederAppliedVolts = feederAppliedVolts.getValueAsDouble();
    inputs.feederCurrentAmps = feederCurrentAmps.getValueAsDouble();
    inputs.feederTempCelsius = feederTempCelsius.getValueAsDouble();

    inputs.shooterRPM =
        shooterVelocityRotPerSec.getValueAsDouble() * 60 / ShooterConstants.GEAR_RATIO;
    inputs.shooterAppliedVolts = shooterAppliedVolts.getValueAsDouble();
    inputs.shooterCurrentAmps = shooterCurrentAmps.getValueAsDouble();
    inputs.shooterTempCelsius = shooterTempCelsius.getValueAsDouble();
  }

  @Override
  public void setFeederVoltage(double volts) {
    feeder.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setShooterVoltage(double volts) {
    shooter.setControl(voltageRequest.withOutput(volts));
  }
}
