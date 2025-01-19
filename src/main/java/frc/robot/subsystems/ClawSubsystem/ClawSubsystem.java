package frc.robot.subsystems.ClawSubsystem;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClawSubsystem extends SubsystemBase {
    private final EndEffectorIOSparkMax center;

    public ClawSubsystem(EndEffectorIOSparkMax center) {
        this.center = center;
    }

    public void open() {
        center.goToSetpoint(1.0);
    }

    public void close() {
        center.goToSetpoint(-1.0);
        //if encoder stops becauuse of object: set to 0 and rotate small wheels inward
    }
}
