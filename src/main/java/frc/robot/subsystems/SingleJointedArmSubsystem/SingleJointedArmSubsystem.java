package frc.robot.subsystems.SingleJointedArmSubsystem;



import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private final ArmFeedforward armFeedForward = new ArmFeedforward(SingleJointedArmConstants.kS, SingleJointedArmConstants.kG, SingleJointedArmConstants.kV );
    // add soleniod thingy
    private final PIDController armFeedback = new PIDController(SingleJointedArmConstants.kP, SingleJointedArmConstants.kI, SingleJointedArmConstants.kD);

    public SingleJointedArmSubsystem() {
        this.armIOInputs = new ArmIOInputsAutoLogged();
        armFeedback.setTolerance(SingleJointedArmConstants.kArmToleranceRPS);
        //armEncoder.setDistancePerPulse(SingleJointedArmConstants.kEncoderDistancePerPulse);

    }
    @Override
    public void periodic(){
        SingleJointedArmIO.updateInputs(armIOInputs);
    }
    public void setArmVoltage(double volts){
        SingleJointedArmIO.setArmMotorVoltage(volts);
    }
}

