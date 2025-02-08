package frc.robot.constants;
import edu.wpi.first.math.system.plant.DCMotor;

public class SingleJointedArmConstants {
    public static final int MotorNumber = 0;
    public static final int EncoderNumber = 1;
    public static final double kP = 0;
    public static final double kD = 0;
    public static final double kI = 0;
    public static final double kV= 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double armMotorReduction = 0;
    public static final double kSVolts = 0;
    public static final double kVVoltsSecondsPerRotation = 0;
    public static final double kArmToleranceRPS = 0;
    public static final double kEncoderDistancePerPulse = 0;
    public static final double MAX_ANGLE = 3.14159/2;
    public static final double MIN_ANGLE = 0.0;
    public static double SparkKp = 0.0;
    public static double SparkKd = 0.0;
    public static double gearingRatio = 0.1;
    public static double armLength = 2.6;
    public static DCMotor armGearbox = DCMotor.getNEO(1);
    public static int armSparkMotorCurrentLimit = 0;
    public static double armSparkEncoderPositionFactor;
    public static double armSparkEncoderVelocityFactor;
    public static double maxVelocity;
    public static double maxAcceleration;

}