package frc.robot.subsystems.ElevatorSubsystem;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.constants.ElevatorConstants;

public class ElevatorEncoderIOLaserCAN implements ElevatorEncoderIO{

    private final LaserCan laser;
    private double lastTime;
    private double lastDistance;
    LinearFilter elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);


    public ElevatorEncoderIOLaserCAN() {
        laser = new LaserCan(ElevatorConstants.elevatorLaserCanID);
        lastTime = Timer.getFPGATimestamp();
        lastDistance = laser.getMeasurement().distance_mm * 1000;
    }

    @Override
    public void updateInputs(ElevatorEncoderIO.ElevatorEncoderIOInputs inputs) {
        LaserCan.Measurement measurement = laser.getMeasurement();
        inputs.unfilteredLoadHeight = measurement.distance_mm * 1000;
        inputs.loadHeight = inputs.unfilteredLoadHeight;
        inputs.unfiliteredHeightVelocity = (laser.getMeasurement().distance_mm * 1000 - lastDistance) / (Timer.getFPGATimestamp() - lastTime);
        lastTime = Timer.getFPGATimestamp();
        lastDistance = laser.getMeasurement().distance_mm * 1000;
    }
}


