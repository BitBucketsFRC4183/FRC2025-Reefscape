package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
    public ElevatorIO elevatorIO;
    public ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS,ElevatorConstants.kG,ElevatorConstants.kV,ElevatorConstants.kA);
    public final TrapezoidProfile elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5,5));
    public final ProfiledPIDController elevatorFeedback = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new TrapezoidProfile.Constraints(5,5));
    double maxVoltage = 12.0;
    private final ElevatorEncoderIO elevatorEncoderIO;
    private final ElevatorIOInputsAutoLogged elevatorIOInputs;
    private final ElevatorEncoderIOInputsAutoLogged encoderIOInputs;
    // add a method to get profileGoal = new TrapezoidProfile.State(5, 0); based on where you want the robot to switch setpoints to
    //after that, add a method to setpoint = m_profile.calculate(kDt, elevator Heights (L1,L2,etc), profile);
    public ElevatorSubsystem(ElevatorIO elevatorIO, ElevatorEncoderIO elevatorEncoderIO) {
        this.elevatorIO = elevatorIO;
        this.elevatorEncoderIO = elevatorEncoderIO;
        this.elevatorIOInputs = new ElevatorIOInputsAutoLogged();
        this.encoderIOInputs =  new ElevatorEncoderIOInputsAutoLogged();

        elevatorFeedback.setTolerance(ElevatorConstants.kShooterToleranceRPS);
        //elevatorEncoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
        setDefaultCommand(runOnce(elevatorIO::disable).andThen(run(() -> {})).withName("Idle"));
    }

    @Override
    public void periodic(){
        elevatorIO.updateInputs(elevatorIOInputs);
        elevatorEncoderIO.updateInputs(encoderIOInputs);
    }
    public double calculateVolts(double velocity, double positionError){
        return elevatorFeedforward.calculate(velocity) + elevatorFeedback.calculate(positionError);
    }

    public void setElevatorVoltage(double volts) {
        elevatorIO.setElevatorMotorVoltage(volts);
    }
}