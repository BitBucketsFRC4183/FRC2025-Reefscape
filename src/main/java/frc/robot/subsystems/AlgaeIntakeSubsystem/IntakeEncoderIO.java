package frc.robot.subsystems.AlgaeIntakeSubsystem;

public interface IntakeEncoderIO {
    public class IntakeEncoderInputs {
        public static double velocityUnitsPerSec = 0.0;
        public static double distance = 0.0;
    }

    public default double getPivotDistance() { return 0.0; }


}
