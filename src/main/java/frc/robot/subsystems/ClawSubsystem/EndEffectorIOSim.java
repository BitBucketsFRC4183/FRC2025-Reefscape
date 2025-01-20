package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.ClawConstants;

public class EndEffectorIOSim implements EndEffectorIO {

    private final DCMotorSim centerWheel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ClawConstants.bigGearBox, 0.1, ClawConstants.gearing),
            ClawConstants.bigGearBox
    );

    private final DCMotorSim gripperWheel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ClawConstants.smallGearBox, 0.1, ClawConstants.gearing),
            ClawConstants.smallGearBox
    );

    private final EndEffectorEncoderIOSim encoder = new EndEffectorEncoderIOSim();

    public void goToSetpoint(DCMotorSim motor, double setpoint) {
        setVelocity(motor, pidCalculate(encoder, setpoint));
        if (atSetpoint() || encoder.getStopped()) { //motor stops when setpoint is reached or claw closes on object
        setVelocity(motor, 0);
        }
    }

    public void rotateGrippers() {
        setVelocity(gripperWheel, 1);
        try {
            Thread.sleep(30);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void setVelocity(DCMotorSim motor, double velocity) {
        motor.setAngularVelocity(velocity); //probably change later
    }

    public void setVoltage(DCMotorSim motor, double volts) {
        motor.setInputVoltage(volts);
    }
}
