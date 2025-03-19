package frc.robot.subsystems.ArmSubsystem;



import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.OperatorInput;
import frc.robot.constants.Constants;
import frc.robot.constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;


public class ArmSubsystem extends SubsystemBase {
    private ArmIO armIO;
    public ArmFeedforward armFeedForward;
    public final ProfiledPIDController armFeedback;

    private final ArmEncoderIO armEncoderIO;
    private final ArmIOInputsAutoLogged armIOInputs;
    private ArmEncoderIOInputsAutoLogged armEncoderIOInputs = new ArmEncoderIOInputsAutoLogged();
    public double hoverAngle = 6969;
    private final SysIdRoutine sysId;

    public ArmSubsystem(ArmIO armIO, ArmEncoderIO armIOEncoder) {
        if (Constants.currentMode == Constants.Mode.SIM) {
            this.armFeedForward = new ArmFeedforward(ArmConstants.kSSim, ArmConstants.kGSim, ArmConstants.kVSim, ArmConstants.kASim);
            this.armFeedback = new ProfiledPIDController(ArmConstants.kPSim, ArmConstants.kISim, ArmConstants.kDSim, new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration));
        } else {
            this.armFeedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
            this.armFeedback =
                    new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration));
        }

        this.armIO = armIO;
        this.armEncoderIO = armIOEncoder;
        this.armIOInputs = new ArmIOInputsAutoLogged();
        this.armEncoderIOInputs =  new ArmEncoderIOInputsAutoLogged();

        armFeedback.setTolerance(ArmConstants.kArmToleranceRPS);

        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Volts.of(0.5).per(Second),
                                Volts.of(1.5),
                                null,
                                (state) -> Logger.recordOutput("ArmSubsystem/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    }


    @Override
    public void periodic(){
        armIO.updateInputs(armIOInputs);
        armEncoderIO.updateInputs(armEncoderIOInputs);

        Logger.processInputs("ArmSubsystem", armIOInputs);
        Logger.processInputs("ArmSubsystem/encoder", armEncoderIOInputs);
    }

    public double getCurrentAngle(){
        return armEncoderIOInputs.armAngle;
    }

    public void setArmVoltage(double volts){
        double outputVoltage = volts;
        if (OperatorInput.mechanismLimitOverride.getAsBoolean()) {
            outputVoltage = volts;


        } else if ((getCurrentAngle() <= ArmConstants.MIN_ANGLE_RADS)) {
            // if mechanism exceeds limit basically set it to zero
            outputVoltage = Math.signum(outputVoltage) == 1 ? outputVoltage : outputVoltage * 0.07;
        } else if (getCurrentAngle() >= ArmConstants.MAX_ANGLE_RADS) {
            outputVoltage = Math.signum(outputVoltage) == -1 ? outputVoltage : outputVoltage * 0.07;
        } else if (getCurrentAngle() <= ArmConstants.MIN_ANGLE_RADS + Units.degreesToRadians(8)) {
            outputVoltage = Math.signum(outputVoltage) == 1 ? outputVoltage : outputVoltage * 0.33;
        } else if (getCurrentAngle() >= ArmConstants.MAX_ANGLE_RADS - Units.degreesToRadians(8)) {
            outputVoltage = Math.signum(outputVoltage) == -1 ? outputVoltage : outputVoltage * 0.25;
        }

        Logger.recordOutput("ArmSubsystem/outputVoltageAdjusted", outputVoltage);
        armIO.setArmMotorVoltage(outputVoltage);
    }

    public void setArmVoltageCommandBypass(double volts){
        double outputVoltage = volts;
        if (OperatorInput.mechanismLimitOverride.getAsBoolean()) {
            outputVoltage = volts;


        } else if ((getCurrentAngle() <= ArmConstants.MIN_ANGLE_RADS)) {
            // if mechanism exceeds limit basically set it to zero
            outputVoltage = Math.signum(outputVoltage) == 1 ? outputVoltage : outputVoltage * 0.07;
        } else if (getCurrentAngle() >= ArmConstants.MAX_ANGLE_RADS) {
            outputVoltage = Math.signum(outputVoltage) == -1 ? outputVoltage : outputVoltage * 0.07;
        }

        Logger.recordOutput("ArmSubsystem/outputVoltageAdjusted", outputVoltage);
        armIO.setArmMotorVoltage(outputVoltage);
    }

    public void runCharacterization(double output) {
        // bypasses limits, be careful!!!
        armIO.setArmMotorVoltage(output);
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

