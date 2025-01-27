package frc.robot.subsystems.SingleJointedArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.SingleJointedArmConstants;

public class SingleJointedArmIOSim implements SingleJointedArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    public static final SingleJointedArmSim armMotorSim = new SingleJointedArmSim(
            SingleJointedArmConstants.armGearbox,
            SingleJointedArmConstants.gearingRatio,
            0,
            SingleJointedArmConstants.armLength,
            SingleJointedArmConstants.MIN_ANGLE,
            SingleJointedArmConstants.MAX_ANGLE,
            true, 0, 0
            );


}
