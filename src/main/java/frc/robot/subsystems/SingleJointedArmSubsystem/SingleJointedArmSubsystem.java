package frc.robot.subsystems.SingleJointedArmSubsystem;



import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SingleJointedArmConstants;

public class SingleJointedArmSubsystem extends SubsystemBase {
    private final PWMSparkMax armMotor = new PWMSparkMax(SingleJointedArmConstants.MotorNumber);
    private final Encoder armEncoder = new Encoder();
    private final SimpleMotorFeedforward armFeedForward = new SimpleMotorFeedforward(SingleJointedArmConstants.kSVolts, SingleJointedArmConstants.kVVoltsSecondsPerRotation);
    private final PIDController armFeedback = new PIDController(SingleJointedArmConstants.kP, SingleJointedArmConstants.kD, SingleJointedArmConstants.kI);

    public SingleJointedArmSubsystem() {
        armFeedback.setTolerance(SingleJointedArmConstants.kShooterToleranceRPS);
        armEncoder.setDistancePerPulse(SingleJointedArmConstants.kEncoderDistancePerPulse);
        setDefaultCommand(runOnce(() -> {
            armMotor.disable();
        }).andThen(run(() -> {
        })).withName("Idle"));
    }
}

