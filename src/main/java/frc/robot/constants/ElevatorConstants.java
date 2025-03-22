package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;


public class ElevatorConstants {

    public static double kShooterToleranceRPS;

    // rev throughbore 1 rot, per 8096 pulses in 4xmode
    public static double kEncoderDistancePerPulse = 1.0 / 8096.0;
    public static DCMotor elevator1Gearbox = DCMotor.getNEO(1);
    public static DCMotor elevator2Gearbox = DCMotor.getNEO(1);
    public static final int encoderA = 0;
    public static final int encoderB = 1;
    public static final boolean isEncoderReversed = true;

    public static double gearingRatio = 3.0;
    public static final double encoderReduction = 20.7;

    public static double elevatorSparkEncoderPositionFactor = 2 * Math.PI / gearingRatio;
    public static double elevatorSparkEncoderVelocityFactor = 2 * Math.PI / 60 / gearingRatio;


    public static double pulleyRadius = Units.inchesToMeters(2);

    public static final int elevatorMotorCurrentLimit = 50;
    public static double kSSim = 0.0092203;
    public static double kVSim = 1.311;
    public static double kGSim = 0;
    public static double kASim = 0.359;
    public static double kPSim = 0;
    public static double kISim = 0;
    public static double kDSim = 0;

    public static double kS = 0.35852;
    public static double kV = 10.604;
    public static double kG = 0;
    public static double kA = 2.2017;
    public static double kP = 125;
    public static double kI = 0;
    public static double kD = 0;


    public static double L1 = 0;
    public static double L2 = 0.244;
    public static double L3 = 0.3885;
    public static double L4 = 0.810;
    public static double carriageMass = Units.lbsToKilograms(20);
    public static double maxHeight = 1.117;
    public static double minHeight = 0;
    public static double maxVelocity = 3;
    public static double maxAcceleration = 3;

    public static final int elevatorCanID = 10;
    public static final int elevatorSpark1CAN = 14;
    public static final int elevatorSpark2CAN = 15;
    public static final int elevatorLaserCanID = 11;
    public static final boolean elevatorSpark1Inverted = true;
    public static final boolean elevatorSpark2Inverted = false;

}
