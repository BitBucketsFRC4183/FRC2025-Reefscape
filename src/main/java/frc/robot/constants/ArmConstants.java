package frc.robot.constants;
import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {

    public static final int frontLeftDriveCanId = 1;
    public static final int MotorNumber = 0;
    public static final int EncoderNumber = 1;
    public static final double kPSim = 3;
    public static final double kDSim = 2;
    public static final double kISim = 0;
    public static final double kVSim= 0.39;
    public static final double kGSim = 1.696;
    public static final double kSSim = 0;
    public static double kASim = 0;
    public static double kS = 0;
    public static double kV = 1;
    public static double kG = 0;
    public static double kA = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static final double kArmToleranceRPS = 0;
    public static final double kEncoderDistancePerPulse = 0;
    public static final double MAX_ANGLE = Math.PI;
    public static final double MIN_ANGLE = -Math.PI;
    public static double SparkkP = 1.0;
    public static double SparkkD = 1.0;
    public static double gearingRatio = 180.0;
    public static double armLength = 0.3;
    public static DCMotor armGearbox = DCMotor.getNEO(1);
    public static int armSparkMotorCurrentLimit = 1;
    public static double armSparkEncoderPositionFactor;
    public static double armSparkEncoderVelocityFactor;
    public static double maxVelocity = 3;
    public static double maxAcceleration = 3;
    public static double mass = 5;

    public static int arm1CurrentLimit = 0;
    public static int arm2CurrentLimit = 0;

    public static int arm1TalonID;
    public static int arm2TalonID;

    public static int fullRange = 4;
    public static int expectedZero = 2;
}