package frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    public final SingleJointedArmSim armMotorSim = new SingleJointedArmSim(
            ArmConstants.armGearbox,
            ArmConstants.gearingRatio,
            SingleJointedArmSim.estimateMOI(ArmConstants.armLength, ArmConstants.mass),
            ArmConstants.armLength,
            ArmConstants.MIN_ANGLE_RADS,
            ArmConstants.MAX_ANGLE_RADS,
            true, -Math.PI/2, 0.01,0
            );

    @Override
    public void updateInputs(ArmIO.ArmIOInputs inputs) {
        armMotorSim.update(LOOP_PERIOD_SECS);
        inputs.arm1CurrentAmps = Math.abs(armMotorSim.getCurrentDrawAmps());
        inputs.arm1AppliedVolts = armMotorSim.getInput(0);
    }
    @Override
    public void setArmMotorVoltage(double volts) {
        armMotorSim.setInputVoltage(volts);
    }

}
