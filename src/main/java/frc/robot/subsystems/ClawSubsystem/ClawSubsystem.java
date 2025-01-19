package frc.robot.subsystems.ClawSubsystem;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClawSubsystem extends SubsystemBase {
    private Trigger button;
    private final EndEffectorEncoderIO encoder = new EndEffectorEncoderIOSim();
    private final EndEffectorIO center = new EndEffectorIOSparkMax(new SparkMax(0, SparkLowLevel.MotorType.kBrushless), encoder);

    public void open() {
        center.goToSetpoint(1.0);
    }

    public Command close() {
        center.goToSetpoint(-1.0);
    }
}
