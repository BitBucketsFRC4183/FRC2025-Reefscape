package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClawConstants {

    public static final int centralID = 29;
    public static final int wheelID = 32;
    public final static DCMotor bigGearBox = DCMotor.getNEO(1);
    public final static DCMotor smallGearBox = DCMotor.getNEO(1);
    public  final static double gearing = 0.1;
    public final static double kP = 0.0;
    public final static double kI = 0.0;
    public final static double kD = 0.0;
    public static double mainSetpoint = 1.0;
}
