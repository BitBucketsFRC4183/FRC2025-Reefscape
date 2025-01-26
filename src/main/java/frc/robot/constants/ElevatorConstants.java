package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;


public class ElevatorConstants {

    public static double kDt = 0.02;
    public static int kElevatorMotorPort;
    public static int[] kEncoderPorts = new int[1];
    public static boolean kEncoderReversed;
    public static double kShooterToleranceRPS;
    public static double kEncoderDistancePerPulse;
    public static DCMotor elevator1Gearbox = DCMotor.getNEO(1);
    public static DCMotor elevator2Gearbox = DCMotor.getNEO(1);
    public static double SparkP = 1;
    public static double SparkD = 1 ;
    public static int elevatorSparkMotorCurrentLimit = 1 ;
    public static double elevatorSparkEncoderPositionFactor = 1 ;
    public static double elevatorSparkEncoderVelocityFactor =1;
    public static double gearingRatio = 10.0;
    public static double pulleyRadius = Units.inchesToMeters(2);
    public static double kS = 0;
    public static double kV = 3.96;
    public static double kG = 0.92;
    public static double kA = 0.5;
    public static double kP = 12;
    public static double kI = 0;
    public static double kD = 0;
    public static double L1 = 0.635; // THESE VALUES ARE IN Meters. FOR EVERY CALCULATION, ASSUME THESE ARE INCHES.
    public static double L2 = 0.635;
    public static double L3 = 1.04775;
    public static double L4 = 1.7018;
    public static int elevatorSparkCAN1 = 4;
    public static int elevatorSparkCAN2 = 3;
    public static double carriageMass = 4.0;
    public static double maxHeight = 2.0;
    public static double minHeight = 0;
    public static double maxVelocity = 3;
    public static double maxAcceleration = 3 ;
}
