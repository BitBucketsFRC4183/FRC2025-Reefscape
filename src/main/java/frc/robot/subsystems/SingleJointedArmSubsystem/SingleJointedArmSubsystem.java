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




public class SingleJointedArmSubsystem extends SubsystemBase {
    public static int canID;
    public SingleJointedArmIO singleJointedArm;
    private final ArmIOInputsAutoLogged armIOInputs;
    private final SparkMax armMotor = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
    //private final Encoder armEncoder = new Encoder();
    public final ArmFeedforward armFeedForward = new ArmFeedforward(SingleJointedArmConstants.kS, SingleJointedArmConstants.kG, SingleJointedArmConstants.kV );
    // add soleniod thingy
    public final ProfiledPIDController armFeedback = new ProfiledPIDController(SingleJointedArmConstants.kP, SingleJointedArmConstants.kI, SingleJointedArmConstants.kD, new TrapezoidProfile.Constraints(SingleJointedArmConstants.maxVelocity,SingleJointedArmConstants.maxAcceleration));

    public SingleJointedArmSubsystem() {
        this.armIOInputs = new ArmIOInputsAutoLogged();
        armFeedback.setTolerance(SingleJointedArmConstants.kArmToleranceRPS);
        //armEncoder.setDistancePerPulse(SingleJointedArmConstants.kEncoderDistancePerPulse);

    }
    @Override
    public void periodic(){
        singleJointedArm.updateInputs(armIOInputs);
    }

    public double getCurrentAngle(){
        return armIOInputs.armAngle;
    }
    public void setArmVoltage(double volts){
        singleJointedArm.setArmMotorVoltage(volts);
    }
}

