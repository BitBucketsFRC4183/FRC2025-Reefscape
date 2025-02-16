package frc.robot.subsystems.GroundIntakeSubsystem;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import static edu.wpi.first.units.Units.Meters;

public class IntakeIOSim implements IntakeIO{
    private final SwerveDriveSimulation driveSim;
    private final IntakeSimulation intakeSim;


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
        } else {
            intakeSim.stopIntake();
        }
    }

    @Override
    public boolean coralInside() {
        return intakeSim.getGamePiecesAmount() != 0;
    }


}
