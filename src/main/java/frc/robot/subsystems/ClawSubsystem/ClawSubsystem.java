package frc.robot.subsystems.ClawSubsystem;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClawSubsystem extends SubsystemBase {
    private Trigger button;
    private final EndEffectorEncoderIOSim encoder = new EndEffectorEncoderIOSim();
    private final EndEffectorIOSparkMax center = new EndEffectorIOSparkMax(new SparkMax(0, SparkLowLevel.MotorType.kBrushless), encoder);

    public Command openCommand() {
        return this.runOnce(() -> center.goToSetpoint(1.0));
    }

    public Command closeCommand() {
        return this.runOnce(() -> center.goToSetpoint(-1.0));
    }
}
