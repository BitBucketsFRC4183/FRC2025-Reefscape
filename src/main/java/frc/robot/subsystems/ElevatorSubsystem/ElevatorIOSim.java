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

    public static final ElevatorSim elevatorMotor1Sim = new ElevatorSim(
            ElevatorConstants.elevator1Gearbox,
            ElevatorConstants.gearingRatio,
            ElevatorConstants.carriageMass,
            ElevatorConstants.pulleyRadius,
            ElevatorConstants.minHeight,
            ElevatorConstants.maxHeight,
            true,
            0,
            0.01,0);

    public static final DigitalInput toplimitSwitch = new DigitalInput(0);
    public static final DigitalInput bottomlimitSwitch = new DigitalInput(1);

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        elevatorMotor1Sim.update(LOOP_PERIOD_SECS);
        inputs.unfilteredLoadHeight = elevatorMotor1Sim.getPositionMeters();
        inputs.loadHeight = elevatorFilter.calculate(inputs.unfilteredLoadHeight);
        inputs.elevatorCurrentAmps = new double[]{Math.abs(elevatorMotor1Sim.getCurrentDrawAmps())};


        Logger.recordOutput("ElevatorSubsystem/toplimitSwitch", toplimitSwitch.get());
        Logger.recordOutput("ElevatorSubsystem/bottomlimitSwitch", bottomlimitSwitch.get());
        Logger.recordOutput("ElevatorSubsystem/loadHeight", inputs.loadHeight);
        Logger.recordOutput("ElevatorSubsystem/unfilteredLoadHeight", inputs.unfilteredLoadHeight);
    }
    @Override
    public void setElevatorMotorVoltage(double volts) {
        double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotor1Sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setEncoderHeightValue(double height) {
        elevatorMotor1Sim.setState(0, 0);
    }
}
