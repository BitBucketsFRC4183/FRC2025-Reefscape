package frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
<<<<<<< Updated upstream
import frc.robot.constants.ElevatorConstants
=======
import frc.robot.Constants;
>>>>>>> Stashed changes

public class ElevatorSubsystem extends SubsystemBase {
    private static final double maxRotationalSpeed = Units.feetToMeters(0);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0,0,0,0);
    private final PIDController elevatorFeedback = new PIDController(ElevatorConstants.kP, 0.0, 0.0);
    private Double speedSetpoint = null;

    private final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorPort);
    private final Encoder elevatorEncoder = new Encoder(ElevatorConstants.kEncoderPorts[0], ElevatorConstants.kEncoderPorts[1],ElevatorConstants.kEncoderReversed);

    public ElevatorSubsystem() {
<<<<<<< Updated upstream
        elevatorFeedback.setTolerance(ElevatorConstants.kShooterToleranceRPS);
        elevatorEncoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
=======
        elevatorFeedback.setTolerance(Constants.elevatorConstants.kShooterToleranceRPS);
        elevatorEncoder.setDistancePerPulse(elevatorConstants.kEncoderDistancePerPulse);
>>>>>>> Stashed changes

        setDefaultCommand(runOnce(() ->{
            elevatorMotor.disable();
        }).andThen(run(() -> {})).withName("Idle"));;
    }
}