// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * This is a sample program to demonstrate the use of a BangBangController with a flywheel to
 * control RPM.
 */
public class Shooter extends TimedRobot {
  private static final int kMotorPort = 20;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;

  // Max setpoint for joystick control in RPM
  private static final double kMaxSetpointValue = 6000.0;

  // Joystick to control setpoint
  private final Joystick m_joystick = new Joystick(0);
  // Flywheel motor controllers (three Kraken motors). Update ports if your hardware is different.
  private final PWMSparkMax m_flywheelMotor1 = new PWMSparkMax(kMotorPort);
  private static final int kMotorPort2 = 21;
  private static final int kMotorPort3 = 22;
  private final PWMSparkMax m_flywheelMotor2 = new PWMSparkMax(kMotorPort2);
  private final PWMSparkMax m_flywheelMotor3 = new PWMSparkMax(kMotorPort3);
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

  private final BangBangController m_bangBangController = new BangBangController();

  // Gains are for example purposes only - must be determined for your own robot!
  public static final double kFlywheelKs = 0.0001; // V
  public static final double kFlywheelKv = 0.000195; // V/RPM
  public static final double kFlywheelKa = 0.0003; // V/(RPM/s)
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(kFlywheelKs, kFlywheelKv, kFlywheelKa);

  // Simulation classes help us simulate our robot

  // Reduction between motors and encoder, as output over input. If the flywheel
  // spins slower than the motors, this number should be greater than one.
  private static final double kFlywheelGearing = 1.0;

  // 1/2 MR²
  private static final double kFlywheelMomentOfInertia =
      0.5 * Units.lbsToKilograms(1.5) * Math.pow(Units.inchesToMeters(4), 2);
  public static final byte ODOMETRY_FREQUENCY = 0;
  // Lock used by odometry threads/operations. Use ReentrantLock so callers can call
  // lock()/unlock().
  public static final Lock odometryLock = new ReentrantLock();

  // Model the flywheel in simulation as three Kraken X60 FOC motors
  private final DCMotor m_gearbox = DCMotor.getKrakenX60Foc(3);

  private final LinearSystem<N1, N1, N1> m_plant =
      LinearSystemId.createFlywheelSystem(m_gearbox, kFlywheelGearing, kFlywheelMomentOfInertia);

  private final FlywheelSim m_flywheelSim = new FlywheelSim(m_plant, m_gearbox);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  public Shooter() {
    // Add bang-bang controller to SmartDashboard and networktables.
    SmartDashboard.putData(m_bangBangController);
  }

  /** Controls flywheel to a set speed (RPM) controlled by a joystick. */
  @Override
  public void teleopPeriodic() {
    // Scale setpoint value between 0 and maxSetpointValue
    double setpoint =
        Math.max(
            0.0,
            m_joystick.getRawAxis(0)
                * Units.rotationsPerMinuteToRadiansPerSecond(kMaxSetpointValue));

    // Set setpoint and measurement of the bang-bang controller
    double bangOutput = m_bangBangController.calculate(m_encoder.getRate(), setpoint) * 12.0;

    // Controls the flywheel with the output of the BangBang controller and a
    // feedforward. Send the same voltage to all three Kraken motors.
    double outputVolts = bangOutput + 0.9 * m_feedforward.calculate(setpoint);
    m_flywheelMotor1.setVoltage(outputVolts);
    m_flywheelMotor2.setVoltage(outputVolts);
    m_flywheelMotor3.setVoltage(outputVolts);
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated velocities to our simulated encoder
    // Use the average controller output when supplying the simulation
    double avgOutput =
        (m_flywheelMotor1.get() + m_flywheelMotor2.get() + m_flywheelMotor3.get()) / 3.0;
    m_flywheelSim.setInputVoltage(avgOutput * RobotController.getInputVoltage());
    m_flywheelSim.update(0.02);
    m_encoderSim.setRate(m_flywheelSim.getAngularVelocityRadPerSec());
  }
}
