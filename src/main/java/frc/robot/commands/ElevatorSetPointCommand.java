package frc.robot.commands;

import com.revrobotics.spark.SparkBase;
<<<<<<< HEAD
import edu.wpi.first.math.controller.PIDController;
=======
>>>>>>> origin/elevator
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIOSparkMax;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;

public class ElevatorSetPointCommand  extends Command {
<<<<<<< HEAD
    private final double setpoint = 0;
=======
    private final double setpoint;
>>>>>>> origin/elevator
    private final ElevatorIOSparkMax elevatorIOSparkMax = new ElevatorIOSparkMax(ElevatorConstants.elevatorSpark1,ElevatorConstants.elevatorSpark2,  ElevatorIOSparkMax.timestampQueue,ElevatorIOSparkMax.elevatorPositionQueue) ;
    public ElevatorSubsystem elevator = new ElevatorSubsystem(elevatorIOSparkMax);
    // private final ElevatorIOSparkMax sparkMaxencoder;
    // Constructor with parameters
    public ElevatorSetPointCommand(ElevatorSubsystem elevator, int setpointnumber) {
        this.elevator = elevator;
        addRequirements(elevator);
<<<<<<< HEAD
        if (setpointnumber == 1) {
            elevator.profileGoal = new TrapezoidProfile.State(ElevatorConstants.L1, 0);
            elevator.profileSetPoint = elevator.elevatorProfile.calculate(ElevatorConstants.kDt, elevator.profileSetPoint, elevator.profileGoal);}
        else if (setpointnumber == 2){
            elevator.profileGoal = new TrapezoidProfile.State(ElevatorConstants.L3, 0);
            elevator.profileSetPoint = elevator.elevatorProfile.calculate(ElevatorConstants.kDt, elevator.profileSetPoint, elevator.profileGoal);}
        else if (setpointnumber == 3){
            elevator.profileGoal = new TrapezoidProfile.State(ElevatorConstants.L4, 0);
            elevator.profileSetPoint = elevator.elevatorProfile.calculate(ElevatorConstants.kDt, elevator.profileSetPoint, elevator.profileGoal);}}
=======
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
>>>>>>> origin/elevator

    public void execute(){
        elevator.MoveElevatorToSetpoint(int profileSetPoint) {
        }
    }

    public boolean isFinished() {
        return elevator.elevatorFeedback.atSetpoint();
    }
}



<<<<<<< HEAD
t
=======
>>>>>>> origin/elevator
