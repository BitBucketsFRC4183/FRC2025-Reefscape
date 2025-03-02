package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import org.opencv.core.Mat;

public class IntakeConstants {
    public static DCMotor pivotGearBox = DCMotor.getNEO(1);
    public static DCMotor rollersGearBox = DCMotor.getNEO(1);
    public final static double kG = 0;
    public static final double kGSim = 0;
    public static double gearing = 10;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    // magnitude of the voltage to run rollers at
    public static double rollerVoltsTarget = 5.0;
    public static final boolean rollerInverted = false;

    // scaling factor for manual control of algae intake

    public static double intakeVoltageFactor = 8;
    public static int intakeMotorCurrentLimit = 40;
    public static double pivotSparkEncoderPositionFactor = 2 * Math.PI / gearing;
    public static double pivotSparkEncoderVelocityFactor= 2 * Math.PI / 60 / gearing;
    public static int pivotID = 12;
    public static int rollerID = 13;
}
