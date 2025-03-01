package frc.robot.subsystems.AlgaeIntakeSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.IntakeConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import static edu.wpi.first.units.Units.Meters;

public class IntakeIOSim implements IntakeIO{
    private final SwerveDriveSimulation driveSim;
    private final IntakeSimulation intakeSim;
    private boolean isRunning = false;
    private final IntakeEncoderIOSim encoder = new IntakeEncoderIOSim();

    private final DCMotorSim pivotWheel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(IntakeConstants.pivotGearBox, 0.1, IntakeConstants.gearing),
            IntakeConstants.pivotGearBox
    );

    private final DCMotorSim rollers = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(IntakeConstants.rollersGearBox, 0.1, IntakeConstants.gearing),
            IntakeConstants.rollersGearBox
    );

    public IntakeIOSim(SwerveDriveSimulation driveSim) {
        this.driveSim = driveSim;
        this.intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Coral",
                driveSim,
                Meters.of(0.4),
                Meters.of(0.2),
                IntakeSimulation.IntakeSide.BACK, //where intake is attatched
                1);
    }

    @Override
    public void setRunning(boolean state) {
        if (state) {
            intakeSim.startIntake();
            rollers.setInputVoltage(IntakeConstants.rollerVoltsTarget);
            isRunning = true;
        } else {
            intakeSim.stopIntake();
            rollers.setInputVoltage(0.0);
            isRunning = false;
        }
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotWheel.setInputVoltage(volts);
    }

    @Override
    public void setRollersVoltage(double volts) {
        rollers.setInputVoltage(volts);
    }

    @Override
    public boolean coralInside() {
        return intakeSim.getGamePiecesAmount() != 0;
    }

    @Override
    public boolean getIsRunning() {
        return isRunning;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.hasCoral = coralInside();
        inputs.isRunning = getIsRunning();
        inputs.rollersVoltage = rollers.getInputVoltage();
        inputs.pivotVoltage = pivotWheel.getInputVoltage();
    }


}
