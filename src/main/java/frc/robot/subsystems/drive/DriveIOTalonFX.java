package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class DriveIOTalonFX implements DriveIO {
  private final TalonFX leftLead;
  private final TalonFX leftFollower;
  private final TalonFX rightLead;
  private final TalonFX rightFollower;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftCurrent;

  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightCurrent;

  private final Debouncer leftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rightConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public DriveIOTalonFX() {
    leftLead = new TalonFX(DriveConstants.kLeftLeadMotorId);
    leftFollower = new TalonFX(DriveConstants.kLeftFollowerMotorId);
    rightLead = new TalonFX(DriveConstants.kRightLeadMotorId);
    rightFollower = new TalonFX(DriveConstants.kRightFollowerMotorId);

    // Configure left lead
    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted =
        DriveConstants.kLeftMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = DriveConstants.kGearRatio;
    leftConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kDriveKP)
            .withKI(DriveConstants.kDriveKI)
            .withKD(DriveConstants.kDriveKD)
            .withKS(DriveConstants.kDriveKS)
            .withKV(DriveConstants.kDriveKV);
    tryUntilOk(5, () -> leftLead.getConfigurator().apply(leftConfig, 0.25));

    // Configure right lead
    var rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.Inverted =
        DriveConstants.kRightMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    rightConfig.Feedback.SensorToMechanismRatio = DriveConstants.kGearRatio;
    rightConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kDriveKP)
            .withKI(DriveConstants.kDriveKI)
            .withKD(DriveConstants.kDriveKD)
            .withKS(DriveConstants.kDriveKS)
            .withKV(DriveConstants.kDriveKV);
    tryUntilOk(5, () -> rightLead.getConfigurator().apply(rightConfig, 0.25));

    // Configure followers
    leftFollower.setControl(new Follower(leftLead.getDeviceID(), MotorAlignmentValue.Aligned));
    rightFollower.setControl(new Follower(rightLead.getDeviceID(), MotorAlignmentValue.Aligned));

    // Status signals
    leftPosition = leftLead.getPosition();
    leftVelocity = leftLead.getVelocity();
    leftAppliedVolts = leftLead.getMotorVoltage();
    leftCurrent = leftLead.getStatorCurrent();

    rightPosition = rightLead.getPosition();
    rightVelocity = rightLead.getVelocity();
    rightAppliedVolts = rightLead.getMotorVoltage();
    rightCurrent = rightLead.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftPosition, rightPosition, leftVelocity, leftAppliedVolts, leftCurrent, rightPosition, rightVelocity, rightAppliedVolts, rightCurrent);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    var leftStatus = BaseStatusSignal.refreshAll(leftPosition, leftVelocity, leftAppliedVolts, leftCurrent);
    var rightStatus = BaseStatusSignal.refreshAll(rightPosition, rightVelocity, rightAppliedVolts, rightCurrent);

    inputs.leftConnected = leftConnectedDebounce.calculate(leftStatus.isOK());
    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();

    inputs.rightConnected = rightConnectedDebounce.calculate(rightStatus.isOK());
    inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftLead.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setRightVoltage(double volts) {
    rightLead.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setLeftVelocity(double velocityRadPerSec) {
    leftLead.setControl(velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setRightVelocity(double velocityRadPerSec) {
    rightLead.setControl(velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }
}
