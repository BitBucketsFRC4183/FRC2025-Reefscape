package frc.robot.subsystems.SingleJointedArmSubsystem;



import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands.ArmHoverCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIOInputsAutoLogged;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmIO;


public class SingleJointedArmSubsystem extends SubsystemBase {
    private SingleJointedArmIO singleJointedArmIO;
    public ArmFeedforward armFeedForward;
    public final ProfiledPIDController armFeedback;

    private final ArmEncoderIO armEncoderIO;
    private final ArmIOInputsAutoLogged armIOInputs;
    private ArmEncoderIOInputsAutoLogged armEncoderIOInputs = new ArmEncoderIOInputsAutoLogged();
    public double hoverAngle = 6969;

    public SingleJointedArmSubsystem(SingleJointedArmIO singleJointedArmIO, ArmEncoderIO armIOEncoder) {
        if (Constants.currentMode == Constants.Mode.SIM) {
            this.armFeedForward = new ArmFeedforward(SingleJointedArmConstants.kSSim, SingleJointedArmConstants.kGSim, SingleJointedArmConstants.kVSim, SingleJointedArmConstants.kASim);
            this.armFeedback = new ProfiledPIDController(SingleJointedArmConstants.kPSim, SingleJointedArmConstants.kISim, SingleJointedArmConstants.kDSim, new TrapezoidProfile.Constraints(SingleJointedArmConstants.maxVelocity, SingleJointedArmConstants.maxAcceleration));
        } else {
            this.armFeedForward = new ArmFeedforward(SingleJointedArmConstants.kS, SingleJointedArmConstants.kG, SingleJointedArmConstants.kV, SingleJointedArmConstants.kA);
            this.armFeedback = new ProfiledPIDController(SingleJointedArmConstants.kP, SingleJointedArmConstants.kI, SingleJointedArmConstants.kD, new TrapezoidProfile.Constraints(SingleJointedArmConstants.maxVelocity, SingleJointedArmConstants.maxAcceleration));
        }

        this.singleJointedArmIO = singleJointedArmIO;
        this.armEncoderIO = armIOEncoder;
        this.armIOInputs = new ArmIOInputsAutoLogged();
        this.armEncoderIOInputs =  new ArmEncoderIOInputsAutoLogged();

//        this.encoderIOInputs = new SingleJointedArmIOEncoderInputs();
        armFeedback.setTolerance(SingleJointedArmConstants.kArmToleranceRPS);
        //armEncoder.setDistancePerPulse(SingleJointedArmConstants.kEncoderDistancePerPulse);
    }


    @Override
    public void periodic(){
        singleJointedArmIO.updateInputs(armIOInputs);
        armEncoderIO.updateInputs(armEncoderIOInputs);
    }

    public void resetArmAngleEncoderValue() {
        armEncoderIO.resetEncoderPositionWithArmAngle();
    }
    public double getCurrentAngle(){
        return armEncoderIOInputs.armAngle;
    }
    public double getArmSpeedRads() {
        return armEncoderIOInputs.encoderVelocityRads;
    }
    public void setArmVoltage(double volts){
        singleJointedArmIO.setArmMotorVoltage(volts);
    }
}

