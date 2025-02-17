package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ManualArmCommand extends Command {
    public SingleJointedArmSubsystem singleJointedArmSubsystem;
    DoubleSupplier yStickDistanceSupplier;

    public ManualArmCommand(SingleJointedArmSubsystem singleJointedArmSubsystem, DoubleSupplier yStickDistanceSupplier) {
        this.singleJointedArmSubsystem = singleJointedArmSubsystem;
        addRequirements(singleJointedArmSubsystem);
        this.yStickDistanceSupplier = yStickDistanceSupplier;
    }

    @Override
    public void execute() {
        double joystickY = yStickDistanceSupplier.getAsDouble() * -1;

        double calculatedVolts = SingleJointedArmConstants.kV / 1.5 * joystickY + SingleJointedArmConstants.kG * Math.cos(singleJointedArmSubsystem.getCurrentAngle());
        Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
        singleJointedArmSubsystem.hoverAngle = singleJointedArmSubsystem.getCurrentAngle();
        this.singleJointedArmSubsystem.setArmVoltage(calculatedVolts);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            double calculatedVolts = SingleJointedArmConstants.kG;
            Logger.recordOutput("ArmSubsystem/target_voltage", calculatedVolts);
            this.singleJointedArmSubsystem.setArmVoltage(calculatedVolts);
        }
    }
}
