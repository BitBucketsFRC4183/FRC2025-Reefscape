package frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private final EndEffectorIOSparkMax center;
    private final EndEffectorIO.EndEffectorInputs inputs = new EndEffectorIO.EndEffectorInputs();

    public ClawSubsystem(EndEffectorIOSparkMax center) {
        this.center = center;
    }

    public void open() {
        center.goToSetpoint(1.0);
    }

    public void close() {
        center.goToSetpoint(-1.0);
        //if encoder stops becauuse of object: set to 0 and rotate small wheels inward
    }

    public void robotPeriodic() {
        center.updateInputs(inputs);
    }
}
