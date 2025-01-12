package frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

public class ElevatorIOTalonFX implements ElevatorIO{
    private final TalonFX elevatorTalon1;
    private final TalonFX getElevatorTalon2;
    private final RelativeEncoder elevatorEncoder;

    public ElevatorIOTalonFX(TalonFX talon1, TalonFX talon2, RelativeEncoder encoder) {
        elevatorTalon1 = talon1;
        getElevatorTalon2 = talon2;
        elevatorEncoder = encoder;
    }

    }
