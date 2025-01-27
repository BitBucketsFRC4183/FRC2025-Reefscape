package frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ClawSubsystem extends SubsystemBase {
    private final EndEffectorIO endEffector;
    private final EndEffectorIO.EndEffectorInputsAutoLogged inputs = new EndEffectorIO.EndEffectorInputsAutoLogged();

    public ClawSubsystem(EndEffectorIO endEffector) {
        this.endEffector = endEffector;
    }
    public void open() {
        endEffector.centralToSetpoint(ClawConstants.mainSetpoint); //rotate big wheel at center
        endEffector.setHasCoral(false);
        endEffector.setIsOpen(true);
    }

    public void close() {
        endEffector.centralToSetpoint(-ClawConstants.mainSetpoint);
        endEffector.grippersToSetpoint(0.2); //rotate grippers for better hold
        endEffector.setHasCoral(true); //assumes the object is always coral
        endEffector.setIsOpen((false));

    }

    @Override
    public void periodic() {
        endEffector.updateInputs(inputs);
    }
}

