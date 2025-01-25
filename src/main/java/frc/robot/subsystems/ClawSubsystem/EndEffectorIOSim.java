package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.ClawConstants;

public class EndEffectorIOSim implements EndEffectorIO {

    private final DCMotorSim centralWheel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ClawConstants.bigGearBox, 0.1, ClawConstants.gearing),
            ClawConstants.bigGearBox
    );

    private final DCMotorSim gripperWheels = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ClawConstants.smallGearBox, 0.1, ClawConstants.gearing),
            ClawConstants.smallGearBox
    );

    private final EndEffectorEncoderIOSim encoder = new EndEffectorEncoderIOSim();

    public void centralToSetpoint(double setpoint) {
        setCentralVelocity(pidCalculate(encoder, setpoint));
    }

    public void grippersToSetpoint(double setpoint) {
        setGrippersVelocity(pidCalculate(encoder, setpoint));
    }

    public void setCentralVelocity(double velocity) {
        centralWheel.setAngularVelocity(velocity); //probably change later
    }

    public void setCentralVoltage(double volts) {
        centralWheel.setInputVoltage(volts);
    }

    public void setGrippersVelocity(double velocity) {
        gripperWheels.setAngularVelocity(velocity); //probably change later
    }

    public void setGrippersVoltage(double volts) {
        gripperWheels.setInputVoltage(volts);
    }

    public void updateInputs(EndEffectorInputs inputs) {
        //inputs.volts = ._.;
    }
}
