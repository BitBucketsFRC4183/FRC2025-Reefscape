package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import org.littletonrobotics.junction.Logger;

public class BendCommand extends Command {

    private final ArmSubsystem armSubsystem;
    public double targetAngle;

    public BendCommand(ArmSubsystem subsystem, double targetAngle) {
        this.armSubsystem = subsystem;
        this.targetAngle = targetAngle;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.armFeedback.reset(armSubsystem.getCurrentAngle());
        armSubsystem.armFeedback.setGoal(targetAngle);

        Logger.recordOutput("ArmSubsystem/target_Angle", targetAngle);

    }

    @Override
    public void execute() {
        double voltsPID = armSubsystem.armFeedback.calculate(armSubsystem.getCurrentAngle());
        double calculatedVolts = armSubsystem.armFeedForward.calculate(armSubsystem.armFeedback.getSetpoint().position, armSubsystem.armFeedback.getSetpoint().velocity) + voltsPID;
        armSubsystem.hoverAngle = armSubsystem.getCurrentAngle();
        Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
        Logger.recordOutput("ArmSubsystem/desired_position", armSubsystem.armFeedback.getSetpoint().position);

        this.armSubsystem.setArmVoltage(calculatedVolts);
    }

}
