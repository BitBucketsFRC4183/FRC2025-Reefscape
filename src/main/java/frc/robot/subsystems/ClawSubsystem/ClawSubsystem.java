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
        endEffector.setCentralVoltage(ClawConstants.mainVoltageTarget); //rotate big wheel at center
        endEffector.setIsOpen(true);
    }

    public void close() {
        endEffector.setCentralVoltage(ClawConstants.mainVoltageTarget);
        endEffector.setGrippersVoltage(ClawConstants.grippersVoltageTarget); //rotate grippers for better hold
        endEffector.setIsOpen(false);

    }

    @Override
    public void periodic() {
        endEffector.updateInputs(inputs);
    }
}

