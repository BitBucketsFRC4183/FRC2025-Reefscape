package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import org.littletonrobotics.junction.Logger;

public class ArmHoverCommand extends Command {
    public ArmSubsystem armSubsystem;

    public ArmHoverCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.armFeedback.reset(armSubsystem.getCurrentAngle());
        armSubsystem.armFeedback.setGoal(armSubsystem.hoverAngle);
    }
    @Override
    public void execute(){
        if (armSubsystem.hoverAngle != 6969) {
            double appliedVolts = (
                    armSubsystem.armFeedForward.calculate(armSubsystem.hoverAngle, 0) +
                            armSubsystem.armFeedback.calculate(armSubsystem.getCurrentAngle()));
            armSubsystem.hoverAngle = armSubsystem.getCurrentAngle();
            Logger.recordOutput("ArmSubsystem/target_voltage", appliedVolts);
            Logger.recordOutput("ArmSubsystem/target_Angle", armSubsystem.hoverAngle);
            armSubsystem.setArmVoltageCommandBypass(appliedVolts);
        }

    }
}
