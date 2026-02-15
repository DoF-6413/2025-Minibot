package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotStateConstants;

public class FeederIOTalonFX implements FeederIO {
    // Motor, controller, configurator
    private final TalonFX m_feederTalonFX;
    private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

    // Status signals
    private StatusSignal<Voltage> m_appliedVolts;
    private StatusSignal<Current> m_currentAmps;
    private StatusSignal<Temperature> m_tempCelsius;
    private StatusSignal<Angle> m_positionRot;
    private StatusSignal<AngularVelocity> m_velocityRotPerSec;

    // Constructor
    public FeederIOTalonFX() {
        System.out.println("[INIT] FeederIOTalonFX");

        m_feederTalonFX = new TalonFX(FeederConstants.CAN_ID);

        m_motorConfig.MotorOutput.withInverted(FeederConstants.IS_INVERTED ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
        .withNeutralMode(FeederConstants.IS_BRAKE_MODE_ENABLED ? NeutralModeValue.Brake : NeutralModeValue.Coast)
        .withControlTimesyncFreqHz(FeederConstants.UPDATE_FREQUENCY_HZ);

        m_feederTalonFX.setPosition(0.0);
        m_feederTalonFX.optimizeBusUtilization();
        m_feederTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

        m_motorConfig.CurrentLimits.withStatorCurrentLimit(FeederConstants.CURRENT_LIMIT).withStatorCurrentLimitEnable(FeederConstants.ENABLE_CURRENT_LIMIT);

        m_feederTalonFX.getConfigurator().apply(m_motorConfig);

        // Update IOs
        m_positionRot = m_feederTalonFX.getPosition();
        m_velocityRotPerSec = m_feederTalonFX.getVelocity();
        m_appliedVolts = m_feederTalonFX.getMotorVoltage();
        m_currentAmps = m_feederTalonFX.getStatorCurrent();
        m_tempCelsius = m_feederTalonFX.getDeviceTemp();

        m_positionRot.setUpdateFrequency(FeederConstants.UPDATE_FREQUENCY_HZ);
        m_velocityRotPerSec.setUpdateFrequency(FeederConstants.UPDATE_FREQUENCY_HZ);
        m_appliedVolts.setUpdateFrequency(FeederConstants.UPDATE_FREQUENCY_HZ);
        m_currentAmps.setUpdateFrequency(FeederConstants.UPDATE_FREQUENCY_HZ);
        m_tempCelsius.setUpdateFrequency(FeederConstants.UPDATE_FREQUENCY_HZ);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.isOK =
          BaseStatusSignal.refreshAll(
                  m_positionRot,
                  m_velocityRotPerSec,
                  m_appliedVolts,
                  m_currentAmps,
                  m_tempCelsius)
              .isOK();

        inputs.appliedVoltage = m_appliedVolts.getValueAsDouble();
        inputs.currentAmps = m_currentAmps.getValueAsDouble();
        inputs.tempCelsius = m_tempCelsius.getValueAsDouble();
        inputs.relativePosRad = m_positionRot.getValueAsDouble();
        inputs.absPositionRad = m_positionRot.getValueAsDouble();
        inputs.velocityRadPerSec = m_velocityRotPerSec.getValueAsDouble();
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        m_feederTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setVoltage(double volts) {
        m_feederTalonFX.setVoltage(MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
    }
}