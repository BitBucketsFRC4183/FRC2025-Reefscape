package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmSubsystem;

public class BendCommand extends Command{

    private final SingleJointedArmSubsystem m_singleJointedArmSubsystem;

    public BendCommand(SingleJointedArmSubsystem subsystem){
        m_singleJointedArmSubsystem = new SingleJointedArmSubsystem();
    }

    public void initialize(){

    }

    public void execute(){

    }

    public boolean isFinished(){
        return true;
    }
}
