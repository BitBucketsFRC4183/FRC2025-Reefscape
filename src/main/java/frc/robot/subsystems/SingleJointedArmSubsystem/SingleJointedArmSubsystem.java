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


public class SingleJointedArmSubsystem extends SubsystemBase {
    public SingleJointedArmIO singleJointedArmIO;
    private final ArmIOInputsAutoLogged armIOInputs;
    //private final Encoder armEncoder = new Encoder();
    public double hoverAngle = -Math.PI / 2;
    public final ArmFeedforward armFeedForward = new ArmFeedforward(SingleJointedArmConstants.kS, SingleJointedArmConstants.kG, SingleJointedArmConstants.kV, 0);
    public final ProfiledPIDController armFeedback = new ProfiledPIDController(SingleJointedArmConstants.kP, SingleJointedArmConstants.kI, SingleJointedArmConstants.kD, new TrapezoidProfile.Constraints(SingleJointedArmConstants.maxVelocity,SingleJointedArmConstants.maxAcceleration));

    public SingleJointedArmSubsystem(SingleJointedArmIO singleJointedArmIO) {
        this.singleJointedArmIO = singleJointedArmIO;
        this.armIOInputs = new ArmIOInputsAutoLogged();

        armFeedback.setTolerance(SingleJointedArmConstants.kArmToleranceRPS);
        //armEncoder.setDistancePerPulse(SingleJointedArmConstants.kEncoderDistancePerPulse);
    }

    @Override
    public void periodic(){
        singleJointedArmIO.updateInputs(armIOInputs);
    }

    public double getCurrentAngle(){
        return armIOInputs.armAngle;
    }

    public void setArmVoltage(double volts){
        singleJointedArmIO.setArmMotorVoltage(volts);
    }

    public double getCurrentVelocity() {
        return armIOInputs.armVelocity;
    }
}

