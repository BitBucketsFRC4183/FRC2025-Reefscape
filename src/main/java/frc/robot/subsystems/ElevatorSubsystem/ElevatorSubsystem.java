package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.OperatorInput;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;


public class ElevatorSubsystem extends SubsystemBase {
    public ElevatorIO elevatorIO;
    public ElevatorFeedforward elevatorFF;
    public final ProfiledPIDController elevatorPID;

    double maxVoltage = 12.0;
    private final ElevatorEncoderIO elevatorEncoderIO;
    private final ElevatorIOInputsAutoLogged elevatorIOInputs;
    private final ElevatorEncoderIOInputsAutoLogged encoderIOInputs;


    private final SysIdRoutine sysId;
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

        elevatorPID.setTolerance(0);
        // elevatorEncoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
        // setDefaultCommand(runOnce(elevatorIO::disable).andThen(run(() -> {})).withName("Idle"));
        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Volts.of(0.5).per(Second),
                                Volts.of(4.5),
                                null,
                                (state) -> Logger.recordOutput("ElevatorSubsystem/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }


    @Override
    public void periodic(){
        elevatorIO.updateInputs(elevatorIOInputs);
        elevatorEncoderIO.updateInputs(encoderIOInputs);

        Logger.processInputs("ElevatorSubsystem", elevatorIOInputs);
        Logger.processInputs("ElevatorSubsystem/encoder", encoderIOInputs);

        // elevatorIO.setElevatorMotorVoltage(ElevatorConstants.kG);
    }

    public void resetLoadHeightEncoderValue() {
        elevatorEncoderIO.resetEncoderPositionWithLoadHeight();
    }
    public double getLoadHeight() {
        return encoderIOInputs.height;
    }
    public double getElevatorHeightSpeed() { return encoderIOInputs.velocity;}

    public void setElevatorVoltage(double volts) {
        double outputVoltage = volts;
        if (OperatorInput.mechanismLimitOverride.getAsBoolean()) {
            outputVoltage = volts;
        } else if ((getLoadHeight() <= ElevatorConstants.minHeight)) {
            outputVoltage = Math.signum(outputVoltage) == 1 ? outputVoltage : ElevatorConstants.kG - 0.1;
        } else if (getLoadHeight() >= ElevatorConstants.maxHeight) {
            outputVoltage = Math.signum(outputVoltage) == -1 ? outputVoltage : ElevatorConstants.kG - 0.05;
        } else if ((getLoadHeight() <= ElevatorConstants.minHeight + 0.10)) {
            outputVoltage = Math.signum(outputVoltage) == 1 ? outputVoltage : outputVoltage * 0.333;
        } else if (getLoadHeight() >= ElevatorConstants.maxHeight - 0.06) {
            outputVoltage = Math.signum(outputVoltage) == -1 ? outputVoltage : outputVoltage * 0.333;

        }
        elevatorIO.setElevatorMotorVoltage(outputVoltage);
    }

    public void setElevatorVoltageCommandBypass(double volts) {
        double outputVoltage = volts;
        if (OperatorInput.mechanismLimitOverride.getAsBoolean()) {
            outputVoltage = volts;
        } else if ((getLoadHeight() <= ElevatorConstants.minHeight)) {
            outputVoltage = Math.signum(outputVoltage) == 1 ? outputVoltage : ElevatorConstants.kG - 0.1;
        } else if (getLoadHeight() >= ElevatorConstants.maxHeight) {
            outputVoltage = Math.signum(outputVoltage) == -1 ? outputVoltage : ElevatorConstants.kG - 0.05;
        } else if ((getLoadHeight() <= ElevatorConstants.minHeight + 0.03)) {
            outputVoltage = Math.signum(outputVoltage) == 1 ? outputVoltage : outputVoltage * 0.333;
        } else if (getLoadHeight() >= ElevatorConstants.maxHeight - 0.005) {
            outputVoltage = Math.signum(outputVoltage) == -1 ? outputVoltage : outputVoltage * 0.333;

        }
        elevatorIO.setElevatorMotorVoltage(outputVoltage);
    }

    public void runCharacterization(double output) {
        // bypasses limits, be careful!!!
        elevatorIO.setElevatorMotorVoltage(output);
    }
    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }
}