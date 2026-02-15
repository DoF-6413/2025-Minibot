package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.RobotStateConstants;

public class FeederIOSim implements FeederIO{
    private final SingleJointedArmSim m_armSim;

    public FeederIOSim() {
        System.out.println("[INIT] Creating PivotIOSim");

        m_armSim = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX60(1), FeederConstants.MOI_KG_M2, FeederConstants.GEAR_RATIO), null, FeederConstants.GEAR_RATIO, FeederConstants.LENGTH_M, FeederConstants.MIN_ANGLE_RAD, FeederConstants.MAX_ANGLE_RAD, false, FeederConstants.STOW_ANGLE_RAD, null);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        m_armSim.update(RobotStateConstants.PERIODIC_LOOP_SEC);
    }
}
