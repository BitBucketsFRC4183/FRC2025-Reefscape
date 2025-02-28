package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;


public class ElevatorSubsystem extends SubsystemBase {
    public ElevatorIO elevatorIO;
    public ElevatorFeedforward elevatorFF;
    public final ProfiledPIDController elevatorPID;

    double maxVoltage = 12.0;
    private final ElevatorEncoderIO elevatorEncoderIO;
    private final ElevatorIOInputsAutoLogged elevatorIOInputs;
    private final ElevatorEncoderIOInputsAutoLogged encoderIOInputs;

    private final Mechanism2d elevator2D = new Mechanism2d(2, 2);
    private final MechanismRoot2d elevator2dRoot = elevator2D.getRoot("Elevator Root", 1, 0);
    private final MechanismLigament2d elevatorMech2d;
    // add a method to get profileGoal = new TrapezoidProfile.State(5, 0); based on where you want the robot to switch setpoints to
    //after that, add a method to setpoint = m_profile.calculate(kDt, elevator Heights (L1,L2,etc), profile);
    public ElevatorSubsystem(ElevatorIO elevatorIO, ElevatorEncoderIO elevatorEncoderIO) {

        if (Constants.currentMode == Constants.Mode.SIM) {
            this.elevatorFF = new ElevatorFeedforward(ElevatorConstants.kSSim, ElevatorConstants.kGSim, ElevatorConstants.kVSim, ElevatorConstants.kASim);
            this.elevatorPID = new ProfiledPIDController(ElevatorConstants.kPSim, ElevatorConstants.kISim, ElevatorConstants.kDSim, new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration));
        } else {
            this.elevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
            this.elevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration));
        }

        this.elevatorIO = elevatorIO;
        this.elevatorEncoderIO = elevatorEncoderIO;
        this.elevatorIOInputs = new ElevatorIOInputsAutoLogged();
        this.encoderIOInputs =  new ElevatorEncoderIOInputsAutoLogged();
        this.elevatorMech2d = elevator2dRoot.append(new MechanismLigament2d("Elevator", encoderIOInputs.loadHeight, 90));
        elevatorPID.setTolerance(0.001, ElevatorConstants.kShooterToleranceRPS);
        //elevatorEncoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
        setDefaultCommand(runOnce(elevatorIO::disable).andThen(run(() -> {})).withName("Idle"));
        SmartDashboard.putData("ElevatorSubsystem/mechanism", elevator2D);

    }


    @Override
    public void periodic(){
        elevatorIO.updateInputs(elevatorIOInputs);
        elevatorEncoderIO.updateInputs(encoderIOInputs);

        elevatorMech2d.setLength(encoderIOInputs.loadHeight);
        Logger.processInputs("ElevatorSubsystem", elevatorIOInputs);
        Logger.processInputs("ElevatorSubsystem/encoder", encoderIOInputs);

    }

    public void resetLoadHeightEncoderValue() {
        elevatorEncoderIO.resetEncoderPositionWithLoadHeight();
    }
    public double getLoadHeight() {
        return encoderIOInputs.loadHeight;
    }
    public double getElevatorSpeedRads() {
        return encoderIOInputs.encoderVelocityRads;
    }
    public void setElevatorVoltage(double volts) {
        elevatorIO.setElevatorMotorVoltage(volts);
    }
}