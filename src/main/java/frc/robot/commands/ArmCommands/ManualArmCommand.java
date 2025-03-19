package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ManualArmCommand extends Command {
    public ArmSubsystem armSubsystem;
    DoubleSupplier yStickDistanceSupplier;
    public ManualArmCommand(ArmSubsystem armSubsystem, DoubleSupplier yStickDistanceSupplier) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
        this.yStickDistanceSupplier = yStickDistanceSupplier;
    }

    @Override
    public void execute() {
        double joystickY = yStickDistanceSupplier.getAsDouble() * 1;
        double calculatedVolts = armSubsystem.armFeedForward.calculate(armSubsystem.getCurrentAngle(), joystickY * ArmConstants.maxVelocity * 0.6);
        Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
        armSubsystem.hoverAngle = armSubsystem.getCurrentAngle();
        this.armSubsystem.setArmVoltage(calculatedVolts);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            double calculatedVolts = ArmConstants.kG * Math.cos(armSubsystem.getCurrentAngle());
            Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
            this.armSubsystem.setArmVoltage(calculatedVolts);
        }
    }
}
