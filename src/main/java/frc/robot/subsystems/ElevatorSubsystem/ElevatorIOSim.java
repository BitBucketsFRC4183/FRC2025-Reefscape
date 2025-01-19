package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ElevatorSetPointCommand;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

import static frc.robot.constants.ElevatorConstants.pulleyRadius;


public class ElevatorIOSim implements ElevatorIO {


    private static final double LOOP_PERIOD_SECS = 0.02;

    private final DCMotorSim elevatorMotor1Sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ElevatorConstants.elevator1Gearbox, 0.025, ElevatorConstants.elevator1MotorReduction),
            ElevatorConstants.elevator1Gearbox);
    private final DCMotorSim elevatorMotor2Sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ElevatorConstants.elevator2Gearbox, 0.025, ElevatorConstants.elevator2MotorReduction),
            ElevatorConstants.elevator2Gearbox);

    private double elevatorAppliedVolts;


    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        elevatorMotor1Sim.update(LOOP_PERIOD_SECS);
        elevatorMotor2Sim.update(LOOP_PERIOD_SECS);
        Logger.recordOutput("ElevatorSubsystem/position", inputs.loadHeight);
        inputs.loadHeight = (2 * Math.PI * pulleyRadius) * ((elevatorMotor1Sim.getAngularPositionRad() - inputs.lastEncoderPosition) * ElevatorConstants.gearingRatio);
        inputs.lastEncoderPosition = elevatorMotor1Sim.getAngularPositionRad();

        inputs.elevatorAppliedVolts = elevatorAppliedVolts;
        Logger.recordOutput("ElevatorSubsystem/voltage", elevatorAppliedVolts);
        inputs.elevatorCurrentAmps = new double[] {Math.abs(elevatorMotor1Sim.getCurrentDrawAmps())};
    }

    @Override
    public void setBothElevatorMotorVoltages(double volts) {
        elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotor1Sim.setInputVoltage(elevatorAppliedVolts);
        elevatorMotor2Sim.setInputVoltage(elevatorAppliedVolts);
    }
}
