package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotContainer;
import frc.robot.constants.ClawConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import static edu.wpi.first.units.Units.Meters;

public class EndEffectorIOSim implements EndEffectorIO {
    private boolean hasAlgae = false;
    private boolean hasCoral = false;
    private boolean isOpen;
    private final SwerveDriveSimulation driveSimulation; //remove placeholder
    private final IntakeSimulation intakeSimulation;

    private final DCMotorSim centralWheel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ClawConstants.bigGearBox, 0.1, ClawConstants.gearing),
            ClawConstants.bigGearBox
    );

    private final DCMotorSim gripperWheels = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ClawConstants.smallGearBox, 0.1, ClawConstants.gearing),
            ClawConstants.smallGearBox
    );

    final PIDController pid = new PIDController(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD);

    public EndEffectorIOSim(SwerveDriveSimulation driveSimulation) {
        this.driveSimulation = driveSimulation;
        intakeSimulation = IntakeSimulation.OverTheBumperIntake("Coral",
                driveSimulation,
                Meters.of(ClawConstants.intakeWidth),
                Meters.of(ClawConstants.extensionLengthBeyondFrame),
                IntakeSimulation.IntakeSide.BACK,
                1);
        setupPID(pid, 3.0, 5.0, -0.5, 0.5); //change pid settings
    }

    private final EndEffectorEncoderIOSim encoder = new EndEffectorEncoderIOSim();

    public boolean getHasCoral() { return this.hasCoral; }

    public boolean getHasAlgae() { return this.hasAlgae; }

    public boolean getIsOpen() { return this.isOpen; }

    @Override
    public void setHasCoral(boolean setting) { this.hasCoral = setting; }

    @Override
    public void setHasAlgae(boolean setting) { this.hasAlgae = setting; }

    @Override
    public void setIsOpen(boolean setting) { this.isOpen = setting; }

    @Override
    public void centralToSetpoint(double setpoint) { //move wheels to setpoint
        setCentralVelocity(pidCalculate(pid, encoder, setpoint));
        if (setpoint == -ClawConstants.mainSetpoint) { //closing
            setHasCoral(true);
        } else if (setpoint == ClawConstants.mainSetpoint) { //opening
            setHasCoral(false); //assmumes object is always coral
        }

    }

    @Override
    public void grippersToSetpoint(double setpoint) { setGrippersVelocity(pidCalculate(pid, encoder, setpoint)); }

    @Override
    public void setCentralVelocity(double velocity) {
        centralWheel.setAngularVelocity(velocity); //probably change later
    }

    @Override
    public void setGrippersVelocity(double velocity) {
        gripperWheels.setAngularVelocity(velocity); //probably change later
    }

    @Override
    public void setCentralVoltage(double volts) {
        centralWheel.setInputVoltage(volts);
    }

    @Override
    public void setGrippersVoltage(double volts) {
        gripperWheels.setInputVoltage(volts);
    }

    @Override
    public void updateInputs(EndEffectorInputsAutoLogged inputs) {
        inputs.centralVolts = centralWheel.getInputVoltage();
        inputs.gripperVolts = gripperWheels.getInputVoltage();
        inputs.hasCoral = getHasAlgae();
        inputs.hasAlgae = getHasCoral();
        inputs.isOpen = getIsOpen();
    }
}
