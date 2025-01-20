package frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ElevatorSetPointCommand;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

import static frc.robot.constants.ElevatorConstants.pulleyRadius;


public class ElevatorIOSim implements ElevatorIO {
    ElevatorIO elevatorIO = new ElevatorIO() {};
    ElevatorEncoderIO elevatorEncoderIO = new ElevatorEncoderIO() {};
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevatorIO, elevatorEncoderIO);
    private static final double LOOP_PERIOD_SECS = 1;

    private final ElevatorSim elevatorMotor1Sim = new ElevatorSim(
            ElevatorConstants.elevator1Gearbox,
            ElevatorConstants.elevator1MotorReduction,
            ElevatorConstants.carriageMass,
            pulleyRadius,
            ElevatorConstants.minHeight,
            ElevatorConstants.maxHeight,
            true,0,0,0);

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        elevatorMotor1Sim.update(LOOP_PERIOD_SECS);
        inputs.loadHeight = elevatorMotor1Sim.getPositionMeters();
        inputs.elevatorCurrentAmps = new double[]{Math.abs(elevatorMotor1Sim.getCurrentDrawAmps())};
        elevatorMotor1Sim.setInputVoltage(elevatorSubsystem.calculateVolts(elevatorMotor1Sim.getVelocityMetersPerSecond(),elevatorMotor1Sim.getPositionMeters() - inputs.loadHeight));

        Logger.recordOutput("ElevatorSubsystem/loadHeight", inputs.loadHeight);
    }
    @Override
    public void setElevatorMotorVoltage(double volts) {
        double elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotor1Sim.setInputVoltage(elevatorAppliedVolts);
    }
}
