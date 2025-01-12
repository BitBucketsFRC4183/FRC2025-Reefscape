package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ElevatorSetPointCommand  extends Command {
    public int setpoint;
    public ElevatorSubsystem elevator;
    // Constructor with parameters
    public ElevatorSetPointCommand(ElevatorSubsystem elevator, int setpoint) {
        this.setpoint = setpoint;
        this.elevator = elevator;
    }

    public void execute(int setpoint){
        elevator.MoveElevator(setpoint);
    }

}

