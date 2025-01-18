package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorConstants {
    public static double kP;
    public static int kElevatorMotorPort;
    public static int[] kEncoderPorts = new int[1];
    public static boolean kEncoderReversed;
    public static double kShooterToleranceRPS;
    public static double kEncoderDistancePerPulse;
    public static DCMotor elevator1Gearbox;
    public static double elevator1MotorReduction;
    public static DCMotor elevator2Gearbox;
    public static double elevator2MotorReduction;
    public static double SparkkP;
    public static double SparkkD;
    public static int elevatorSparkMotorCurrentLimit;
    public static double elevatorSparkEncoderPositionFactor;
    public static double elevatorSparkEncoderVelocityFactor;
    public static double gearingRatio;
    public static double pulleyRadius;

}
