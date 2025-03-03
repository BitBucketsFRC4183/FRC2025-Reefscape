package frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    public static final SingleJointedArmSim armMotorSim = new SingleJointedArmSim(
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
        inputs.armAngle = armMotorSim.getAngleRads();
        inputs.armCurrentAmps = new double[]{Math.abs(armMotorSim.getCurrentDrawAmps())};
        inputs.armVelocity = armMotorSim.getVelocityRadPerSec();
        Logger.recordOutput("ArmSubsystem/armAngle", inputs.armAngle);
        Logger.recordOutput("ArmSubsystem/armCurrentAmps", inputs.armCurrentAmps);

    }
    @Override
    public void setArmMotorVoltage(double volts) {
        armMotorSim.setInputVoltage(volts);
        Logger.recordOutput("ArmSubsystem/appliedVolts", volts);
    }

}
