// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.hopper.HopperConstants;

/**
 * This superstructure implementation is for Talon FXs driving motors like the Falon 500, Kraken
 * X44, or Kraken X60.
 */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intake = new TalonFX(IntakeConstants.CAN_ID);
  private final StatusSignal<Angle> intakePositionRot = intake.getPosition();
  private final StatusSignal<AngularVelocity> intakeVelocityRotPerSec = intake.getVelocity();
  private final StatusSignal<Voltage> intakeAppliedVolts = intake.getMotorVoltage();
  private final StatusSignal<Current> intakeCurrentAmps = intake.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public IntakeIOTalonFX() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> intake.getConfigurator().apply(intakeConfig, 0.25));

    var agitatorConfig = new TalonFXConfiguration();
    agitatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    agitatorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.agitatorCurrentLimit;
    agitatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    agitatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> agitator.getConfigurator().apply(agitatorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakePositionRot,
        intakeVelocityRotPerSec,
        intakeAppliedVolts,
        intakeCurrentAmps,
        agitatorPositionRot,
        agitatorVelocityRotPerSec,
        agitatorAppliedVolts,
        agitatorCurrentAmps);
    ParentDevice.optimizeBusUtilizationForAll(intake, agitator);
  }

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakePositionRot,
        intakeVelocityRotPerSec,
        intakeAppliedVolts,
        intakeCurrentAmps,
        agitatorPositionRot,
        agitatorVelocityRotPerSec,
        agitatorAppliedVolts,
        agitatorCurrentAmps);

    inputs.intakePositionRad = Units.rotationsToRadians(intakePositionRot.getValueAsDouble());
    inputs.intakeVelocityRadPerSec =
        Units.rotationsToRadians(intakeVelocityRotPerSec.getValueAsDouble());
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeCurrentAmps.getValueAsDouble();

    inputs.agitatorPositionRad = Units.rotationsToRadians(agitatorPositionRot.getValueAsDouble());
    inputs.agitatorVelocityRadPerSec =
        Units.rotationsToRadians(agitatorVelocityRotPerSec.getValueAsDouble());
    inputs.agitatorAppliedVolts = agitatorAppliedVolts.getValueAsDouble();
    inputs.agitatorCurrentAmps = agitatorCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intake.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setAgitatorVoltage(double volts) {
    agitator.setControl(voltageRequest.withOutput(volts));
  }
}
