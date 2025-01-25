package frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClawSubsystem extends SubsystemBase {
    private final EndEffectorIO endEffector;
    private final EndEffectorIO.EndEffectorInputs inputs = new EndEffectorIO.EndEffectorInputs();

    public ClawSubsystem(EndEffectorIO endEffector) {
        this.endEffector = endEffector;
    }

    public void open() {
        endEffector.centralToSetpoint(ClawConstants.mainSetpoint); //rotate big wheel at center
    }

    public void close() {
        endEffector.centralToSetpoint(-ClawConstants.mainSetpoint);
        endEffector.grippersToSetpoint(0.2);
    }

    public void robotPeriodic() {
        endEffector.updateInputs(inputs);
        Logger.processInputs("ClawSubsystem", (LoggableInputs) inputs);
    }
}

