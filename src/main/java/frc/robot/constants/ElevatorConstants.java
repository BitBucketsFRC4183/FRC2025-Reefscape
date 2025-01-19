package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.Inches;

public class ElevatorConstants {
    public static double kP;
    public static double kDt = 0.02;
    public static int kElevatorMotorPort;
    public static int[] kEncoderPorts = new int[1];
    public static boolean kEncoderReversed;
    public static double kShooterToleranceRPS;
    public static double kEncoderDistancePerPulse;
    public static DCMotor elevator1Gearbox;
    public static double elevator1MotorReduction;
    public static DCMotor elevator2Gearbox;
    public static double elevator2MotorReduction;
    public static double SparkP;
    public static double SparkD;
    public static int elevatorSparkMotorCurrentLimit;
    public static double elevatorSparkEncoderPositionFactor;
    public static double elevatorSparkEncoderVelocityFactor;
    public static double gearingRatio;
    public static double pulleyRadius;
    public static double kS;
    public static double kV;
    public static double kG;
    public static double kA;
    public static double kI;
    public static double kD;
    public static double L1 = 0.635; // THESE VALUES ARE IN Meters. FOR EVERY CALCULATION, ASSUME THESE ARE INCHES.
    public static double L2 = 0.635;
    public static double L3 = 1.04775;
    public static double L4 = 1.7018;
    public static int elevatorSparkCAN1;
    public static int elevatorSparkCAN2;
}
