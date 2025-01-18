package frc.robot.commands;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIOSparkMax;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;

public class ElevatorSetPointCommand  extends Command {
    private final double setpoint;
    private final ElevatorIOSparkMax elevatorIOSparkMax = new ElevatorIOSparkMax(ElevatorConstants.elevatorSpark1,ElevatorConstants.elevatorSpark2,  ElevatorIOSparkMax.timestampQueue,ElevatorIOSparkMax.elevatorPositionQueue) ;
    public ElevatorSubsystem elevator = new ElevatorSubsystem(elevatorIOSparkMax);
    // private final ElevatorIOSparkMax sparkMaxencoder;
    // Constructor with parameters
    public ElevatorSetPointCommand(ElevatorSubsystem elevator, double setpoint) {
        this.setpoint = setpoint;
        this.elevator = elevator;
        addRequirements(elevator);
        if (elevator.elevatorXbox.getAButton()) {
            elevator.profileGoal = new TrapezoidProfile.State(ElevatorConstants.L1, 0);}
            setpoint = elevator.elevatorProfile.calculate(ElevatorConstants.kDt, ElevatorConstants.L1, elevator.profileGoal);
        else if (elevator.elevatorXbox.getXButton()){
            elevator.profileGoal = new TrapezoidProfile.State(ElevatorConstants.L3, 0);}
        else if (elevator.elevatorXbox.getYButton()){
            elevator.profileGoal = new TrapezoidProfile.State(ElevatorConstants.L4, 0);}
        }

    public void intialize(){
        elevator.elevatorFeedback.setSetpoint(setpoint);
    }

    public void execute(){
        // double power = elevator.elevatorFeedback.calculateOutput(elevator.);
        // elevator.moveElevator(power);
    }

    public boolean isFinished() {
        return elevator.elevatorFeedback.atSetpoint();
    }
}



