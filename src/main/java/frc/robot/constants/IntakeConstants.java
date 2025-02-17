package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeConstants {
    public static DCMotor pivotGearBox = new DCMotor(1, 0.5, 0.5, 0.5, 0.2, 1);
    public static DCMotor rollersGearBox = new DCMotor(1, 0.5, 0.5, 0.5, 0.2, 1);
    public static double gearing = 0.1;
    public static double pivotRotation = 0.3;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double rollerVoltsTarget = 1.0;
}
