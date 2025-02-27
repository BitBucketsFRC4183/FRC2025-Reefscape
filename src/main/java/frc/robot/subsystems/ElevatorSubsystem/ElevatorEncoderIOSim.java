package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.constants.ElevatorConstants;

public class ElevatorEncoderIOSim implements ElevatorEncoderIO {
    private ElevatorSim sim;
    LinearFilter elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ElevatorEncoderIOSim(ElevatorSim sim) {
        this.sim = sim;
    }
    @Override
    public void updateInputs(ElevatorEncoderIO.ElevatorEncoderIOInputs inputs) {

        // 1:1 with the shaft
        inputs.unfilteredLoadHeight = sim.getPositionMeters();
        inputs.loadHeight = elevatorFilter.calculate(inputs.unfilteredLoadHeight);
    }

    @Override
    public void resetEncoderPositionWithLoadHeight() {
        sim.setState(0, 0);
    }
}
