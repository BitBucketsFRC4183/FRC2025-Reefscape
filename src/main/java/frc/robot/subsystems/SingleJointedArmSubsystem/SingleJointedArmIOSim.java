package frc.robot.subsystems.SingleJointedArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIO;
import org.littletonrobotics.junction.Logger;

public class SingleJointedArmIOSim implements SingleJointedArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    public static final SingleJointedArmSim armMotorSim = new SingleJointedArmSim(
            SingleJointedArmConstants.armGearbox,
            SingleJointedArmConstants.gearingRatio,
            SingleJointedArmSim.estimateMOI(SingleJointedArmConstants.armLength, SingleJointedArmConstants.mass),
            SingleJointedArmConstants.armLength,
            SingleJointedArmConstants.MIN_ANGLE,
            SingleJointedArmConstants.MAX_ANGLE,
            true, 0, 0.01,0
            );

    @Override
    public void updateInputs(SingleJointedArmIO.ArmIOInputs inputs) {
        armMotorSim.update(LOOP_PERIOD_SECS);
        inputs.armAngle = armMotorSim.getAngleRads();
        inputs.armCurrentAmps = new double[]{Math.abs(armMotorSim.getCurrentDrawAmps())};
        Logger.recordOutput("ArmSubsystem/armAngle", inputs.armAngle);
        Logger.recordOutput("ArmSubsystem/armCurrentAmps", inputs.armCurrentAmps);

    }
    @Override
    public void setArmMotorVoltage(double volts) {
        armMotorSim.setInputVoltage(volts);
        Logger.recordOutput("ArmSubsystem/appliedVolts", volts);
    }

}
