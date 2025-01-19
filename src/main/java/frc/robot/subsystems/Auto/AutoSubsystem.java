package frc.robot.subsystems.Auto;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeManagementSubsystem.AlgaeManagementSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSubsystem extends SubsystemBase {

    public Command drive() {
        return Commands.sequence(
        );
    }

    public Command deposit() {
        return Commands.sequence(
        );
    }

    public Command claw() {
        return Commands.sequence(
        );
    }


    public class AutoRoutine FourL4CoralBottom  {
        AutoRoutine routine =
                new autoRoutine();


        // Initialize
        autoFactory.autoTrajectory W1toW2 =
                routine.trajectory();
        autoFactory.autoTrajectory W2toDeposit =
                routine.trajectory();
        autoFactory.autoTrajectory W2toW3 =
                routine.trajectory();
        autoFactory.autoTrajectory W3toW4 =
                routine.trajectory();
        autoFactory.autoTrajectory W4toClaw =
                routine.trajectory();
        autoFactory.autoTrajectory W4toW5 =
                routine.trajectory();
        autoFactory.autoTrajectory W5toW6 =
                routine.trajectory();
        autoFactory.autoTrajectory W6toDeposit =
                routine.trajectory();
        autoFactory.autoTrajectory W6toW7 =
                routine.trajectory();
        autoFactory.autoTrajectory W7toW8 =
                routine.trajectory();
        autoFactory.autoTrajectory W8toClaw =
                routine.trajectory();
        autoFactory.autoTrajectory W8toW9 =
                routine.trajectory();
        autoFactory.autoTrajectory W9toDeposit =
                routine.trajectory();
        autoFactory.autoTrajectory W9toW10 =
                routine.trajectory();
        autoFactory.autoTrajectory W10toClaw =
                routine.trajectory();
        autoFactory.autoTrajectory W10toW11 =
                routine.trajectory();
        autoFactory.autoTrajectory W11toDeposit =
                routine.trajectory();
    }

        // TODO whatever trajectory plug in

        routine.active().

        onTrue(
                Commands.sequence(
        )
    );
        rigger myTrigger = new Trigger(() -> condition);

        // Safe
    routine.observe(myTrigger).

        onTrue(Commands.print("Foo"));
    routine.active().

        and(myTrigger).

        onTrue(Commands.print("Bar"));

        // Unsafe
    myTrigger.onTrue(Commands.print("Foo"));
    myTrigger.and(routine.active()).

        onTrue(Commands.print("Bar"));
    }


