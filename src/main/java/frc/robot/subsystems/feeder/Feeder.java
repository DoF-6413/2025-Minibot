package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private final FeederIO m_io;
    private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

    /**
     * Constructs a new {@link Feeder} instance.
     *
     * <p>This creates a new Feeder {@link SubsystemBase} object with the given IO implementation
     * which determines whether the methods and inputs are initialized with the real, sim, or replay
     * code.
     *
     * @param io {@link FeederIO} implementation of the current robot mode.
     */

    public Feeder(FeederIO io) {
        System.out.println("[INIT] Feeder");

        // Initialize the IO implementation
        m_io = io;
    }

    @Override
    public void periodic() {
        // Update and log inputs
        m_io.updateInputs(m_inputs);
    }
    
    /** 
     * Sets idle mode of motor
     * 
     * @param enable {@code} true to enable brake mode, {@code} false for coast.
     */
    public void enableBrakeMode(boolean enable) {
        m_io.enableBrakeMode(enable);
    }

    /** 
     * Sets voltage of motor
     * 
     * @param volts A value between [-12, 12]
     */
    public void setVoltage(double volts) {
        m_io.setVoltage(volts);
    }
}
