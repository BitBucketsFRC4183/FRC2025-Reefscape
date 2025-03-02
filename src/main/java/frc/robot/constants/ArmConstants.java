package frc.robot.constants;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {

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
    public static final double MAX_ANGLE = Math.PI / 2;
    public static final double MIN_ANGLE = -Math.PI / 2;
    public static double gearingRatio = 180.0;
    public static double armLength = 0.3;
    public static DCMotor armGearbox = DCMotor.getKrakenX60(1);

    public static double armSparkEncoderPositionFactor;
    public static double armSparkEncoderVelocityFactor;
    public static double maxVelocity = 3;
    public static double maxAcceleration = 3;
    public static double mass = 5;

    public static int arm1CurrentLimit = 20;
    public static int arm2CurrentLimit = 20;

    public static int arm1TalonID;
    public static int arm2TalonID;

    // 0 as a value should be parallel to the floor, think unit circle
    public static final int encoderChannel = 1;

    // offset in rads, this will be subtracted from the offset reading
    // so if encoder is 1.9, offset is 1.9 and not -1.9
    public static final double encoderOffset = 0;
    public static boolean encoderInverted;
    public static double setpointUp = Math.PI / 4;
    public static double setpointDown =  -Math.PI / 4;
    public static InvertedValue arm1Inverted = InvertedValue.Clockwise_Positive;
    public static InvertedValue arm2Inverted = InvertedValue.CounterClockwise_Positive;
}