package frc.robot.subsystems.SingleJointedArmSubsystem;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.AutoLog;

public class SingleJointedArmIOEncoder {
    SparkMax spark = new SparkMax(1,
            SparkLowLevel.MotorType.kBrushless); //TODO!
    @AutoLog
    

}
