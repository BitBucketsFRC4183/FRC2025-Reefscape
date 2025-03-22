package frc.robot.subsystems.ElevatorSubsystem;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ElevatorConstants;

public class ElevatorEncoderIOLaserCAN implements ElevatorEncoderIO{

    private final LaserCan laser;
    private double lastTime;
    private double lastDistance;
    private double lastUnfilteredDistance;
    LinearFilter elevatorFilter = LinearFilter.movingAverage(10);


    public ElevatorEncoderIOLaserCAN() {
        CanBridge.runTCP();
        laser = new LaserCan(ElevatorConstants.elevatorLaserCanID);
        try {
            laser.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS);
            laser.setRangingMode(LaserCanInterface.RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            System.out.print("me when the laser can");
        }

        lastTime = Timer.getFPGATimestamp();
        try {
            lastUnfilteredDistance = getDistanceMeters();
            lastDistance = elevatorFilter.calculate(lastDistance);
        } catch (NullPointerException e) {
            lastDistance = 0;
        }
    }

    @Override
    public void updateInputs(ElevatorEncoderIO.ElevatorEncoderIOInputs inputs) {
        try {
            inputs.unfilteredHeight = getDistanceMeters();
            inputs.height = elevatorFilter.calculate(inputs.unfilteredHeight);
            inputs.unfiliteredVelocity = (inputs.unfilteredHeight - lastUnfilteredDistance) / (Timer.getFPGATimestamp() - lastTime);
            inputs.velocity = (inputs.height - lastDistance) / (Timer.getFPGATimestamp() - lastTime);
            lastTime = Timer.getFPGATimestamp();

            lastUnfilteredDistance = getDistanceMeters();
            lastDistance = inputs.height;
            inputs.encoderConnected = true;
        } catch (NullPointerException e) {
            inputs.encoderConnected = false;
            lastTime = Timer.getFPGATimestamp();
            return;
        }
    }

    private double getDistanceMeters() {
        if (laser.getMeasurement() != null && laser.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return (double) laser.getMeasurement().distance_mm / 1000;
        } else {
            throw new NullPointerException();
        }
    }
}


