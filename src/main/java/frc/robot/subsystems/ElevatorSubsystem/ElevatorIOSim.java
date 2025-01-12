package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.ElevatorConstants;


public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim elevatorMotor1Sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ElevatorConstants.elevator1Gearbox, 0.025, ElevatorConstants.elevator1MotorReduction),
            ElevatorConstants.elevator1Gearbox);
    private final DCMotorSim elevatorMotor2Sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ElevatorConstants.elevator2Gearbox, 0.025, ElevatorConstants.elevator2MotorReduction),
            ElevatorConstants.elevator2Gearbox);
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
