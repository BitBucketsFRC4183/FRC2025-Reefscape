package frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.ElevatorConstants;

import static frc.robot.constants.ElevatorConstants.L1;


public class ElevatorSubsystem extends SubsystemBase {
    public static ElevatorIO elevatorIO;
    public final XboxController elevatorXbox = new XboxController(0); // currently, we've got nothing so don't worry about it George.
    private static final double maxRotationalSpeed = Units.feetToMeters(0);
    public final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS,ElevatorConstants.kG,ElevatorConstants.kV,ElevatorConstants.kA);
    public final PIDController elevatorFeedback = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private Double speedSetpoint = null;
    double maxVoltage = 12.0;
    public final TrapezoidProfile elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0,0));
    public TrapezoidProfile.State profileGoal = new TrapezoidProfile.State();// goal is the place where we want to go after already moving to another level. For motion profiling, we need to convert this to the setpoint or something idk I dont get paid enough for this crap I mean technically it is a -$350 profit//
    public TrapezoidProfile.State profileSetPoint = new TrapezoidProfile.State();
    private final ElevatorEncoderIO elevatorEncoder = (ElevatorEncoderIO) new Encoder(ElevatorConstants.kEncoderPorts[0], ElevatorConstants.kEncoderPorts[1],ElevatorConstants.kEncoderReversed);
    // add a method to get profileGoal = new TrapezoidProfile.State(5, 0); based on where you want the robot to switch setpoints to
    //after that, add a method to setpoint = m_profile.calculate(kDt, elevator Heights (L1,L2,etc), profile);
    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorFeedback.setTolerance(ElevatorConstants.kShooterToleranceRPS);
        //elevatorEncoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);

        setDefaultCommand(runOnce(ElevatorIO::disable).andThen(run(() -> {})).withName("Idle"));
    }

    @Override
    public void periodic(){
        ElevatorEncoderIO.getDistance();
    }

    public void moveElevatorToVelocity(double velocity, TrapezoidProfile.State setpoint) {
        double feedforwardOutput = elevatorFeedforward.calculate(velocity);
        double pidOutput = elevatorFeedback.calculate(ElevatorEncoderIO.getDistance(), setpoint.position);
        double totalOutput = feedforwardOutput + pidOutput;
        ElevatorIO.setBothElevatorMotorVoltages(totalOutput);
    }
}