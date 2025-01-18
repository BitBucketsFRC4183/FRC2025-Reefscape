package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIOSparkMax;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;

public class ElevatorSetPointCommand  extends Command {
    private final double setpoint;
    private final ElevatorSubsystem elevator;
    private final ElevatorIOSparkMax sparkMaxencoder;
    // Constructor with parameters
    public ElevatorSetPointCommand(ElevatorSubsystem elevator, double setpoint) {
        this.setpoint = setpoint;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    public void intialize(){
        elevator.elevatorFeedback.setSetpoint(setpoint);
    }

    public void execute(){
        double power = elevator.elevatorFeedback.calculateOutput(elevator.);
        elevator.moveElevator(power);
    }

    public boolean isFinished() {
        return elevator.elevatorFeedback.atSetpoint();
    }
}

