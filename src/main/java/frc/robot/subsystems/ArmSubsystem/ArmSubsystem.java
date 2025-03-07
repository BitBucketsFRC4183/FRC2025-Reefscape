package frc.robot.subsystems.ArmSubsystem;



import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OperatorInput;
import frc.robot.constants.Constants;
import frc.robot.constants.ArmConstants;
import org.littletonrobotics.junction.Logger;


public class ArmSubsystem extends SubsystemBase {
    private ArmIO armIO;
    public ArmFeedforward armFeedForward;
    public final ProfiledPIDController armFeedback;

    private final ArmEncoderIO armEncoderIO;
    private final ArmIOInputsAutoLogged armIOInputs;
    private ArmEncoderIOInputsAutoLogged armEncoderIOInputs = new ArmEncoderIOInputsAutoLogged();
    public double hoverAngle = 6969;

    public ArmSubsystem(ArmIO armIO, ArmEncoderIO armIOEncoder) {
        if (Constants.currentMode == Constants.Mode.SIM) {
            this.armFeedForward = new ArmFeedforward(ArmConstants.kSSim, ArmConstants.kGSim, ArmConstants.kVSim, ArmConstants.kASim);
            this.armFeedback = new ProfiledPIDController(ArmConstants.kPSim, ArmConstants.kISim, ArmConstants.kDSim, new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration));
        } else {
            this.armFeedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
            this.armFeedback = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration));
        }

        this.armIO = armIO;
        this.armEncoderIO = armIOEncoder;
        this.armIOInputs = new ArmIOInputsAutoLogged();
        this.armEncoderIOInputs =  new ArmEncoderIOInputsAutoLogged();

        armFeedback.setTolerance(ArmConstants.kArmToleranceRPS);
    }


    @Override
    public void periodic(){
        armIO.updateInputs(armIOInputs);
        armEncoderIO.updateInputs(armEncoderIOInputs);

        Logger.processInputs("ArmSubsystem", armIOInputs);
        Logger.processInputs("ArmSubsystem/encoder", armEncoderIOInputs);
    }

    public double getCurrentAngle(){
        return armIOInputs.armAngle;
    }

    public void setArmVoltage(double volts){
        double outputVoltage = volts;
        if (OperatorInput.mechanismLimitOverride.getAsBoolean()) {
            outputVoltage = volts;
        } else if ((getCurrentAngle() <= ArmConstants.MIN_ANGLE_RADS) || (getCurrentAngle() >= ArmConstants.MAX_ANGLE_RADS)) {
            outputVoltage = outputVoltage * 0.1;
        } else if ((getCurrentAngle() <= ArmConstants.MIN_ANGLE_RADS + Units.degreesToRadians(5)) || (getCurrentAngle() >= ArmConstants.MAX_ANGLE_RADS - Units.degreesToRadians(5))) {
            outputVoltage = outputVoltage * 0.333;
        }

        Logger.recordOutput("ArmSubsystem/outputVoltageAdjusted", outputVoltage);
        armIO.setArmMotorVoltage(outputVoltage);
    }
}

