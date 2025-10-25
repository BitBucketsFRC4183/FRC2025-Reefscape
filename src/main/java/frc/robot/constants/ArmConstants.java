package frc.robot.constants;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ArmConstants {

    public static final double kPSim = 4;
    public static final double kDSim = 0;
    public static final double kISim = 0;
    public static final double kVSim= 3.2981;
    public static final double kGSim = 0.05932;
    public static final double kSSim = 0.065489;
    public static double kASim = 0.053023;


    public static double kS = 0.2353;
    public static double kV = 0.75614;
    public static double kG = 0.26263;
    public static double kA = 0.088;
    public static double kP = 4;
    public static double kI = 0;
    public static double kD = 0.5;
    public static final double kArmToleranceRPS = 0;
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(76);
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-86);
    public static double gearingRatio = 180.0;
    public static double armLength = 0.3;
    public static DCMotor armGearbox = DCMotor.getKrakenX60(2);


    public static double armSparkEncoderPositionFactor;
    public static double armSparkEncoderVelocityFactor;
    public static double maxVelocity = 7;
    public static double maxAcceleration = 7;
    public static double mass = 5;

    public static int arm1CurrentLimit = 50;
    public static int arm2CurrentLimit = 50;

    public static int arm1TalonID = 16;
    public static int arm2TalonID = 17;

    public static final int encoderChannel = 2;

    // offset in rots, this will be subtracted from the offset reading
    // so if encoder is 0.8, offset is 0.8 and not 0.8
    public static final double encoderOffsetRots = 0.133;
    public static boolean encoderInverted;
    public static final double armOriginAngle = Units.degreesToRadians(-86);
    public final static double armL1Angle = Units.degreesToRadians(25.5);
    public final static double armL2Angle = Units.degreesToRadians(52);
    public final static double armL3Angle = Units.degreesToRadians(50);
    public final static double armL4Angle = Units.degreesToRadians(70);
    public static InvertedValue arm1Inverted = InvertedValue.Clockwise_Positive;
    public static InvertedValue arm2Inverted = InvertedValue.CounterClockwise_Positive;
}