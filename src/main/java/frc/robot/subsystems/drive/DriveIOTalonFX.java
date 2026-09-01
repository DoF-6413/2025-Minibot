package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj.AnalogEncoder;

public class DriveIOTalonFX implements DriveIO {
  private final TalonFX leftLead;
  private final TalonFX leftFollower;
  private final TalonFX rightLead;
  private final TalonFX rightFollower;

  private final TalonFX FLTurn;
  private final TalonFX FRTurn;
  private final TalonFX BRTurn;
  private final TalonFX BLTurn;

  private final AnalogEncoder FLEncoder;
  private final AnalogEncoder FREncoder;
  private final AnalogEncoder BREncoder;
  private final AnalogEncoder BLEncoder;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);

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

    FLTurn = new TalonFX(DriveConstants.kFLTurnMotorId);
    FRTurn = new TalonFX(DriveConstants.kFRTurnMotorId);
    BRTurn = new TalonFX(DriveConstants.kBRTurnMotorId);
    BLTurn = new TalonFX(DriveConstants.kBLTurnMotorId);

    FLEncoder = new AnalogEncoder(DriveConstants.kFLEncoderPort, 1.0, DriveConstants.kFLZero);
    FREncoder = new AnalogEncoder(DriveConstants.kFREncoderPort, 1.0, DriveConstants.kFRZero);
    BREncoder = new AnalogEncoder(DriveConstants.kBREncoderPort, 1.0, DriveConstants.kBRZero);
    BLEncoder = new AnalogEncoder(DriveConstants.kBLEncoderPort, 1.0, DriveConstants.kBLZero);

    var slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kP)
            .withKI(DriveConstants.kI)
            .withKD(DriveConstants.kD)
            .withKS(DriveConstants.kS)
            .withKV(DriveConstants.kV);

    // Configure left drive lead
    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted =
        DriveConstants.kLeftMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = DriveConstants.kGearRatio;
    leftConfig.Slot0 = slot0;
    tryUntilOk(5, () -> leftLead.getConfigurator().apply(leftConfig, 0.25));

    // Configure right drive lead
    var rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.Inverted =
        DriveConstants.kRightMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    rightConfig.Feedback.SensorToMechanismRatio = DriveConstants.kGearRatio;
    rightConfig.Slot0 = slot0;
    tryUntilOk(5, () -> rightLead.getConfigurator().apply(rightConfig, 0.25));

    // Configure drive followers
    leftFollower.setControl(new Follower(leftLead.getDeviceID(), MotorAlignmentValue.Aligned));
    rightFollower.setControl(new Follower(rightLead.getDeviceID(), MotorAlignmentValue.Aligned));

    // Configure turn motors
    var FLConfig = new TalonFXConfiguration();
    FLConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    FLConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    FLConfig.Feedback.SensorToMechanismRatio = DriveConstants.kTurnRatio;
    FLConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kFLP)
            .withKI(DriveConstants.kFLI)
            .withKD(DriveConstants.kFLD)
            .withKV(DriveConstants.kFLV);
    tryUntilOk(5, () -> FLTurn.getConfigurator().apply(FLConfig, 0.25));
    tryUntilOk(5, ()-> FLTurn.setPosition(FLEncoder.get(), 0.25));

    var FRConfig = new TalonFXConfiguration();
    FRConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    FRConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    FRConfig.Feedback.SensorToMechanismRatio = DriveConstants.kTurnRatio;
    FRConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kFRP)
            .withKI(DriveConstants.kFRI)
            .withKD(DriveConstants.kFRD)
            .withKV(DriveConstants.kFRV);
    tryUntilOk(5, () -> FRTurn.getConfigurator().apply(FRConfig, 0.25));
    tryUntilOk(5, ()-> FRTurn.setPosition(FREncoder.get(), 0.25));

    var BRConfig = new TalonFXConfiguration();
    BRConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    BRConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    BRConfig.Feedback.SensorToMechanismRatio = DriveConstants.kTurnRatio;
    BRConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kBRP)
            .withKI(DriveConstants.kBRI)
            .withKD(DriveConstants.kBRD)
            .withKV(DriveConstants.kBRV);
    tryUntilOk(5, () -> BRTurn.getConfigurator().apply(BRConfig, 0.25));
    tryUntilOk(5, ()-> BRTurn.setPosition(BREncoder.get(), 0.25));

    var BLConfig = new TalonFXConfiguration();
    BLConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    BLConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    BLConfig.Feedback.SensorToMechanismRatio = DriveConstants.kTurnRatio;
    BLConfig.Slot0 =
        new Slot0Configs()
            .withKP(DriveConstants.kBLP)
            .withKI(DriveConstants.kBLI)
            .withKD(DriveConstants.kBLD)
            .withKV(DriveConstants.kBLV);
    tryUntilOk(5, () -> BLTurn.getConfigurator().apply(BLConfig, 0.25));
    tryUntilOk(5, ()-> BLTurn.setPosition(BLEncoder.get(), 0.25));

    // Status signals
    leftPosition = leftLead.getPosition();
    leftVelocity = leftLead.getVelocity();
    leftAppliedVolts = leftLead.getMotorVoltage();
    leftCurrent = leftLead.getStatorCurrent();

    rightPosition = rightLead.getPosition();
    rightVelocity = rightLead.getVelocity();
    rightAppliedVolts = rightLead.getMotorVoltage();
    rightCurrent = rightLead.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        rightPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    var leftStatus =
        BaseStatusSignal.refreshAll(leftPosition, leftVelocity, leftAppliedVolts, leftCurrent);
    var rightStatus =
        BaseStatusSignal.refreshAll(rightPosition, rightVelocity, rightAppliedVolts, rightCurrent);

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
  public void setLeftVelocity(double velocityRPS) {
    leftLead.setControl(velocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setRightVelocity(double velocityRPS) {
    rightLead.setControl(velocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setTurnPositions(double FLAngle, double FRAngle, double BRAngle, double BLAngle) {
    FLTurn.setControl(positionRequest.withPosition(FLAngle));
    FRTurn.setControl(positionRequest.withPosition(FRAngle));
    BRTurn.setControl(positionRequest.withPosition(BRAngle));
    BLTurn.setControl(positionRequest.withPosition(BLAngle));
  }
}
