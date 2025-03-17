package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClawConstants {
    public static DCMotor bigGearBox = DCMotor.getNeo550(1);
    public static DCMotor smallGearBox = DCMotor.getNeo550(1);
    public static double gearing = 0.1;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double mainVoltageTarget = 3.0;
    public static double grippersVoltageTarget = 2.0;
    public static final int centralID = 29;
    public static final int wheelID = 32;
    public static final double centralSparkEncoderPositionFactor = 1.0;
    public static final double centralSparkEncoderVelocityFactor = 1.0;
    public static final int clawMotorCurrentLimit = 20;
    public static final boolean grippersInverted = false;
    public static final double kGSim = 0;
    public static final double kG = 0;
}
