package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorEncoderIOSim implements ElevatorEncoderIO {
    private ElevatorSim sim;
    LinearFilter elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ElevatorEncoderIOSim(ElevatorSim sim) {
        this.sim = sim;
    }
    @Override
    public void updateInputs(ElevatorEncoderIO.ElevatorEncoderIOInputs inputs) {

        // 1:1 with the shaft
        inputs.unfilteredHeight = sim.getPositionMeters();
        inputs.height = elevatorFilter.calculate(inputs.unfilteredHeight);
    }

    @Override
    public void resetEncoderPositionWithLoadHeight() {
        sim.setState(0, 0);
    }
}
