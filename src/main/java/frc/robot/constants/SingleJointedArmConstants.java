package frc.robot.constants;
import edu.wpi.first.math.system.plant.DCMotor;

public class SingleJointedArmConstants {

    public static final int frontLeftDriveCanId = 1;
    public static final int MotorNumber = 0;
    public static final int EncoderNumber = 1;
    public static final double kP = 3;
    public static final double kD = 2;
    public static final double kI = 0;
    public static final double kV= 0.39;
    public static final double kG = 1.696;
    public static final double kS = 0;
    public static final double kArmToleranceRPS = 0;
    public static final double kEncoderDistancePerPulse = 0;
    public static final double MAX_ANGLE = Math.PI;
    public static final double MIN_ANGLE = -Math.PI;
    public static double SparkkP = 1.0;
    public static double SparkkD = 1.0;
    public static double gearingRatio = 20.0;
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

}