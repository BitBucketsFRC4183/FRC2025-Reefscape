package frc.robot.subsystems.SingleJointedArmSubsystem;



import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.SingleJointedArmConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorEncoderIOInputsAutoLogged;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorIOInputsAutoLogged;


public class SingleJointedArmSubsystem extends SubsystemBase {
    public static int canID;
    public SingleJointedArmIO singleJointedArmIO;
    private final ArmIOInputsAutoLogged armIOInputs;
    private final SparkMax armMotor = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
    //private final Encoder armEncoder = new Encoder();
    public final ArmFeedforward armFeedForward = new ArmFeedforward(SingleJointedArmConstants.kS, SingleJointedArmConstants.kG, SingleJointedArmConstants.kV);
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
}

