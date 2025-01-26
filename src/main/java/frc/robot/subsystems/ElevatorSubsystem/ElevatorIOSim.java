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
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final ElevatorSim elevatorMotor1Sim = new ElevatorSim(
            ElevatorConstants.elevator1Gearbox,
            ElevatorConstants.gearingRatio,
            ElevatorConstants.carriageMass,
            ElevatorConstants.pulleyRadius,
            ElevatorConstants.minHeight,
            ElevatorConstants.maxHeight,
            false,
            0,
            0,0);

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        elevatorMotor1Sim.update(LOOP_PERIOD_SECS);
        inputs.loadHeight = elevatorMotor1Sim.getPositionMeters();
        inputs.elevatorCurrentAmps = new double[]{Math.abs(elevatorMotor1Sim.getCurrentDrawAmps())};
        inputs.elevatorSpeed = elevatorMotor1Sim.getVelocityMetersPerSecond();

        Logger.recordOutput("ElevatorSubsystem/loadHeight", inputs.loadHeight);
    }
    @Override
    public void setElevatorMotorVoltage(double volts) {
        double elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotor1Sim.setInputVoltage(volts);
    }

    @Override
    public void setEncoderHeightValue(double height) {
        elevatorMotor1Sim.setState(0, 0);
    }
}
