package frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private final EndEffectorIO endEffector;
    private final EndEffectorIO.EndEffectorInputs inputs = new EndEffectorIO.EndEffectorInputs();

    public ClawSubsystem(EndEffectorIO endEffector) {
        this.endEffector = endEffector;
    }

    public void open() {
        endEffector.goToSetpoint(1.0); //rotate big wheel at center
    }

    public void close() {
        endEffector.goToSetpoint(-1.0);
        endEffector.rotateGrippers();
    }

    public void robotPeriodic() {
        endEffector.updateInputs(inputs);
    }
}
