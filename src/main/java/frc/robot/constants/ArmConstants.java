package frc.robot.constants;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ArmConstants {

    public static final double kPSim = 3;
    public static final double kDSim = 2;
    public static final double kISim = 0;
    public static final double kVSim= 0.39;
    public static final double kGSim = 1.696;
    public static final double kSSim = 0;
    public static double kASim = 0;


    public static double kS = 0.3;
    public static double kV = 7.17;
    public static double kG = 0.2;
    public static double kA = 0.02;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static final double kArmToleranceRPS = 0;
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(76);
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-86);
    public static double gearingRatio = 180.0;
    public static double armLength = 0.3;
    public static DCMotor armGearbox = DCMotor.getKrakenX60(2);


    public static double armSparkEncoderPositionFactor;
    public static double armSparkEncoderVelocityFactor;
    public static double maxVelocity = 3;
    public static double maxAcceleration = 2;
    public static double mass = 5;

    public static int arm1CurrentLimit = 50;
    public static int arm2CurrentLimit = 50;

    public static int arm1TalonID = 16;
    public static int arm2TalonID = 17;

    // 0 as a value should be parallel to the floor, think unit circle
    public static final int encoderChannel = 2;

    // offset in rads, this will be subtracted from the offset reading
    // so if encoder is 0.8, offset is 0.8 and not 0.8
    public static final double encoderOffsetRots = 0.630;
    public static boolean encoderInverted;
    public final static double armL1Angle = Units.degreesToRadians(25.5);
    public final static double armL2Angle = Units.degreesToRadians(25.5);
    public final static double armL3Angle = Units.degreesToRadians(56.4);
    public final static double armL4Angle = Units.degreesToRadians(77);
    public static InvertedValue arm1Inverted = InvertedValue.Clockwise_Positive;
    public static InvertedValue arm2Inverted = InvertedValue.CounterClockwise_Positive;
}