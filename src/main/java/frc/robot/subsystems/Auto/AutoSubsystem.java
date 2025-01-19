package frc.robot.subsystems.Auto;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeManagementSubsystem.AlgaeManagementSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import static choreo.Choreo.loadTrajectory;

public class AutoSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final AutoFactory autoFactory;

    public AutoSubsystem(DriveSubsystem drive) {
        this.drive = drive;
        this.autoFactory = new AutoFactory();
    }
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



    public AutoRoutine createAutoRoutine() {

        var trajectory = loadTrajectory(
                "BitBucketsTrajectory");
        AutoRoutine routine =
                autoFactory.newRoutine(
                        "BitBucketRountine");

        Command autoTrajectory =
                autoFactory.trajectoryCmd(
                        "BitBucketsTrajectory");

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


        // TODO whatever trajectory plug in

        routine.active().onTrue(
                Commands.sequence(
                        W1toW2.cmd(),
                        W2toDeposit.cmd()
                )
                );


        W1toW2.atTime(drive()).onTrue(DriveSubsystem.drive());

        W1toW2.done().onTrue(W2toDeposit.cmd());

        W2toDeposit.active().whileTrue(ClawSubsystem.getReady());

        W2toDeposit.done().onTrue(ClawSubsystem.score());

        // so on so on

        Trigger atW3 = W2toDeposit.done();
        return routine;

        public class trigger {
            trigger robotTrigger =
                    new Trigger(() -> condition);
            // Safe
            routine.observe(robotTrigger).onTrue(Commands.print("?"));
            routine.active().and(robotTrigger).onTrue(Commands.print("?"));

            // Unsafe
            robotTrigger.onTrue(Commands.print("?"));
            robotTrigger.and(routine.active()).onTrue(Commands.print("?"));
        }

    }

    }

