package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClawConstants {
    public static DCMotor bigGearBox = new DCMotor(1, 0.5, 0.5, 0.5, 0.2, 1);
    public static DCMotor smallGearBox = new DCMotor(1, 0.5, 0.5, 0.5, 0.2, 1);
    public static double gearing = 0.1;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double mainVoltageTarget = 2.0;
    public static double grippersVoltageTarget = 1.0;
}
