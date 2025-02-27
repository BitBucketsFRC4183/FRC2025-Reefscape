package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;


public class ElevatorConstants {

    public static double kDt = 0.02;
    public static int kElevatorMotorPort;
    public static int[] kEncoderPorts = new int[1];
    public static boolean kEncoderReversed;
    public static double kShooterToleranceRPS;
    public static double kEncoderDistancePerPulse;
    public static DCMotor elevator1Gearbox = DCMotor.getNEO(1);



    public static double SparkP = 1;
    public static double SparkD = 1 ;
    public static double elevatorSparkEncoderPositionFactor = 1 ;
    public static double elevatorSparkEncoderVelocityFactor =1;


    public static double gearingRatio = 60.0;
    public static double pulleyRadius = Units.inchesToMeters(2);

    public static final int elevatorMotorCurrentLimit = 50;
    public static double kSSim = 0;
    public static double kVSim = 4.1;
    public static double kGSim = 0.92;
    public static double kASim = 0.5;
    public static double kPSim = 0.04;
    public static double kISim = 0;
    public static double kDSim = 0;

    public static double kS = 0;
    public static double kV = 4.1;
    public static double kG = 0.92;
    public static double kA = 0.5;
    public static double kP = 0.04;
    public static double kI = 0;
    public static double kD = 0;


    public static double L1 = 0.635; // THESE VALUES ARE IN Meters. FOR EVERY CALCULATION, ASSUME THESE ARE INCHES.
    public static double L2 = 0.635;
    public static double L3 = 1.04775;
    public static double L4 = 1.7018;
    public static double carriageMass = 4.0;
    public static double maxHeight = Units.inchesToMeters(67);
    public static double minHeight = 0;
    public static double maxVelocity = 1;
    public static double maxAcceleration = 1;

    public static int elevatorCanID = 10;
}
