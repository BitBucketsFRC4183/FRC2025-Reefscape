package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmSubsystem;
import org.littletonrobotics.junction.Logger;

public class ArmHoverCommand extends Command {
    public SingleJointedArmSubsystem singleJointedArmSubsystem;

    public ArmHoverCommand(SingleJointedArmSubsystem singleJointedArmSubsystem){
        this.singleJointedArmSubsystem = singleJointedArmSubsystem;
        addRequirements(singleJointedArmSubsystem);
    }


    @Override
    public void execute(){

        double appliedVolts = (
                singleJointedArmSubsystem.armFeedForward.calculate(singleJointedArmSubsystem.hoverAngle, 0)+
                singleJointedArmSubsystem.armFeedback.calculate(singleJointedArmSubsystem.getCurrentAngle(), singleJointedArmSubsystem.hoverAngle));
        Logger.recordOutput("ArmSubsystem/appliedVolts", appliedVolts);
        Logger.recordOutput("ArmSubsystem/target_Angle", singleJointedArmSubsystem.hoverAngle);
        singleJointedArmSubsystem.setArmVoltage(appliedVolts);

    }
}
