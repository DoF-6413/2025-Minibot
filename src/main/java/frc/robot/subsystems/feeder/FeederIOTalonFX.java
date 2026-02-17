// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.feeder;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.RobotStateConstants;

/**
 * This superstructure implementation is for Talon FXs driving motors like the Falon 500, Kraken
 * X44, or Kraken X60.
 */
public class FeederIOTalonFX implements FeederIO {
  private final TalonFX m_feeder = new TalonFX(FeederConstants.CAN_ID);
  private final StatusSignal<AngularVelocity> feederVelocityRotPerSec = m_feeder.getVelocity();
  private final StatusSignal<Voltage> feederAppliedVolts = m_feeder.getMotorVoltage();
  private final StatusSignal<Current> feederCurrentAmps = m_feeder.getSupplyCurrent();
  private final StatusSignal<Temperature> feederTempCelsius = m_feeder.getDeviceTemp();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public FeederIOTalonFX() {
    var feederConfig = new TalonFXConfiguration();
    feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    feederConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.CURRENT_LIMIT;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    tryUntilOk(
        5,
        () ->
            m_feeder
                .getConfigurator()
                .apply(feederConfig, RobotStateConstants.PHX_CONFIG_TIMEOUT_SEC));

    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotStateConstants.UPDATE_FREQUENCY_HZ,
        feederVelocityRotPerSec,
        feederAppliedVolts,
        feederCurrentAmps,
        feederTempCelsius);

    m_feeder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        feederVelocityRotPerSec, feederAppliedVolts, feederCurrentAmps, feederTempCelsius);

    // Motor rotations -> feeder rotations * 60 sec/min
    inputs.feederRPM = feederVelocityRotPerSec.getValueAsDouble() * 60 / FeederConstants.GEAR_RATIO;
    inputs.feederAppliedVolts = feederAppliedVolts.getValueAsDouble();
    inputs.feederCurrentAmps = feederCurrentAmps.getValueAsDouble();
    inputs.feederTempCelsius = feederTempCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    m_feeder.setControl(
        voltageRequest.withOutput(
            MathUtil.clamp(
                volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE)));
  }
}
