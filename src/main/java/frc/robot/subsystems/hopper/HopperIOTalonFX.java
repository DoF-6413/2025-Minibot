package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOTalonFX implements HopperIO {

    
  private final TalonFX hopper = new TalonFX(HopperConstants.CAN_ID);
  private final StatusSignal<AngularVelocity> hopperVelocityRotPerSec = hopper.getVelocity();
  private final StatusSignal<Voltage> hopperAppliedVolts = hopper.getMotorVoltage();
  private final StatusSignal<Current> hopperCurrentAmps = hopper.getSupplyCurrent();
}
