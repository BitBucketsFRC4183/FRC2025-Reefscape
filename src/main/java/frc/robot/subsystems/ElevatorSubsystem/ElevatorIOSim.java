package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;


public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    LinearFilter elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public final ElevatorSim elevatorMotor1Sim = new ElevatorSim(
            ElevatorConstants.elevator1Gearbox,
            ElevatorConstants.gearingRatio,
            ElevatorConstants.carriageMass,
            ElevatorConstants.pulleyRadius,
            ElevatorConstants.minHeight,
            ElevatorConstants.maxHeight,
            false,
            0,
            0.01,0);


    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        elevatorMotor1Sim.update(LOOP_PERIOD_SECS);

        inputs.elevatorConnected = true;
        inputs.elevatorAppliedVolts = elevatorMotor1Sim.getInput(0);
        inputs.elevatorCurrentAmps = Math.abs(elevatorMotor1Sim.getCurrentDrawAmps());


        // Logger.recordOutput("ElevatorSubsystem/toplimitSwitch", toplimitSwitch.get());
        // Logger.recordOutput("ElevatorSubsystem/bottomlimitSwitch", bottomlimitSwitch.get());
    }
    @Override
    public void setElevatorMotorVoltage(double volts) {
        double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotor1Sim.setInputVoltage(appliedVolts);
    }
}
