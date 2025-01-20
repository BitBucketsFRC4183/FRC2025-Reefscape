package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
    public ElevatorIO elevatorIO;
    public final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS,ElevatorConstants.kG,ElevatorConstants.kV,ElevatorConstants.kA);
    public final PIDController elevatorFeedback = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    double maxVoltage = 12.0;
    public final TrapezoidProfile elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5,5));
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

    public void setElevatorVoltage(double volts) {
        elevatorIO.setElevatorMotorVoltage(volts);
    }
}