package frc.robot.subsystems.SingleJointedArmSubsystem;



import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SingleJointedArmConstants;



public class SingleJointedArmSubsystem extends SubsystemBase {
    public static int canID;
    private final SparkMax armMotor = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
    //private final Encoder armEncoder = new Encoder();
    private final SimpleMotorFeedforward armFeedForward = new SimpleMotorFeedforward(SingleJointedArmConstants.kSVolts, SingleJointedArmConstants.kVVoltsSecondsPerRotation);
    private final PIDController armFeedback = new PIDController(SingleJointedArmConstants.kP, SingleJointedArmConstants.kD, SingleJointedArmConstants.kI);

    public SingleJointedArmSubsystem() {
        armFeedback.setTolerance(SingleJointedArmConstants.kArmToleranceRPS);
        //armEncoder.setDistancePerPulse(SingleJointedArmConstants.kEncoderDistancePerPulse);
        setDefaultCommand(runOnce(() -> {
            armMotor.disable();
        }).andThen(run(() -> {
        })).withName("Idle"));
    }
}

