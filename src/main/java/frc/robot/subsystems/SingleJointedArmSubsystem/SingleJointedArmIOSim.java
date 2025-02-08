package frc.robot.subsystems.SingleJointedArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.SingleJointedArmConstants;
import org.littletonrobotics.junction.Logger;

public class SingleJointedArmIOSim implements SingleJointedArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    LinearFilter armFilter = LinearFilter.singlePoleIIR(0.1,0.02);
    public static final SingleJointedArmSim armMotorSim = new SingleJointedArmSim(
            SingleJointedArmConstants.armGearbox,
            SingleJointedArmConstants.gearingRatio,
            0.1,
            SingleJointedArmConstants.armLength,
            SingleJointedArmConstants.MIN_ANGLE,
            SingleJointedArmConstants.MAX_ANGLE,
            true, 0, 0.02, 4);

    @Override
    public void updateInputs(SingleJointedArmIO.ArmIOInputs inputs){
        armMotorSim.update(LOOP_PERIOD_SECS);
        inputs.unfilteredLoadAngle = armMotorSim.getAngleRads();
        inputs.LoadAngle = armFilter.calculate(inputs.unfilteredLoadAngle);
        inputs.armCurrentAmps = Math.abs(armMotorSim.getCurrentDrawAmps());
        inputs.armSpeed = armMotorSim.getVelocityRadPerSec();
        Logger.recordOutput("SingleJointedArmSubsystem/LoadAngle", inputs.LoadAngle);
        Logger.recordOutput("SingleJointedArmSubsystem/unfilteredLoadAngle", inputs.unfilteredLoadAngle);
    }
    @Override
    public void setArmVoltage(double volts){
        double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        armMotorSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setEncoderAngleValue(double angle){armMotorSim.setState(0,0);}

}
