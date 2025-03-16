package frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem.ArmEncoderIO;


public class ArmEncoderIOSim implements ArmEncoderIO {
    private final SingleJointedArmSim sim;
    public ArmEncoderIOSim(SingleJointedArmSim sim) {
        this.sim = sim;
    }

    @Override
    public void updateInputs(ArmEncoderIO.ArmEncoderIOInputs inputs) {
        inputs.armAngle = sim.getAngleRads();
        inputs.armAngleDegs = Units.radiansToDegrees(inputs.armAngle);
        inputs.armVelocityRads = sim.getVelocityRadPerSec();
    }

}
