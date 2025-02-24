package frc.robot.subsystems.SingleJointedArmSubsystem;



import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands.ArmHoverCommand;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIOInputsAutoLogged;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIO;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIOInputsAutoLogged;

import java.beans.Encoder;

import static frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmIO.ArmIOInputs;


public class SingleJointedArmSubsystem extends SubsystemBase {
    private final SingleJointedArmIO singleJointedArmIO;
    private final SingleJointedArmIOEncoder singleJointedArmIOEncoder;

    private final ArmIOInputsAutoLogged armIOInputs;
//    private final SingleJointedArmIOEncoderInputs encoderIOInputs;

    public double hoverAngle = 6969; //TODO!!!!!!
    public final ArmFeedforward armFeedForward = new ArmFeedforward(SingleJointedArmConstants.kS, SingleJointedArmConstants.kG, SingleJointedArmConstants.kV, 0);
    public final ProfiledPIDController armFeedback = new ProfiledPIDController(SingleJointedArmConstants.kP, SingleJointedArmConstants.kI, SingleJointedArmConstants.kD, new TrapezoidProfile.Constraints(SingleJointedArmConstants.maxVelocity,SingleJointedArmConstants.maxAcceleration));

    public SingleJointedArmSubsystem(SingleJointedArmIO singleJointedArmIO, SingleJointedArmIOEncoder singleJointedArmIOEncoder) {
        this.singleJointedArmIO = singleJointedArmIO;
        this.singleJointedArmIOEncoder = singleJointedArmIOEncoder;

        this.armIOInputs = new ArmIOInputsAutoLogged();
//        this.encoderIOInputs = new SingleJointedArmIOEncoderInputs();

        armFeedback.setTolerance(SingleJointedArmConstants.kArmToleranceRPS);
        //armEncoder.setDistancePerPulse(SingleJointedArmConstants.kEncoderDistancePerPulse);
    }


    @Override
    public void periodic(){
        singleJointedArmIO.updateInputs(ArmIOInputs);
    }

    public double getCurrentAngle(){

        return ArmIOInputs.armAngle;
    }

    public void setArmVoltage(double volts){
        singleJointedArmIO.setArmMotorVoltage(volts);
    }
}
//    public double getCurrentVelocity();

