package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.DriveSubsystem.ModuleIO;

public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim elevatorMotor1Sim = new DCMotorSim(DCMotor.getNEO(2), 6.75, 0.025);
    private final DCMotorSim elevatorMotor2Sim = new DCMotorSim(DCMotor.getNEO(2), 6.75, 0.025);
    private double elevatorAppliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        elevatorMotor1Sim.update(LOOP_PERIOD_SECS);
        elevatorMotor2Sim.update(LOOP_PERIOD_SECS);
        inputs.loadHeight = 0;
        inputs.elevatorAppliedVolts = elevatorAppliedVolts;
        inputs.elevatorCurrentAmps = new double[] {Math.abs(elevatorMotor1Sim.getCurrentDrawAmps())};
    }

    @Override
    public void setBothElevatorMotorVoltages(double volts) {
        elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotor1Sim.setInputVoltage(elevatorAppliedVolts);
        elevatorMotor2Sim.setInputVoltage(elevatorAppliedVolts);
    }
}
