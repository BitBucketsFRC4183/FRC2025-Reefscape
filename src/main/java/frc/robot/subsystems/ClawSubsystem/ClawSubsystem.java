package frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private final EndEffectorIOSparkMax sparkMax;
    private final EndEffectorIO.EndEffectorInputs inputs = new EndEffectorIO.EndEffectorInputs();

    public ClawSubsystem(EndEffectorIOSparkMax sparkMax) {
        this.sparkMax = sparkMax;
    }

    public void open() {
        sparkMax.goToSetpoint(1.0); //rotate big wheel at center
    }

    public void close() {
        sparkMax.goToSetpoint(-1.0);
        //sparkMax.rotateSmall();
    }

    public void robotPeriodic() {
        sparkMax.updateInputs(inputs);
    }
}
