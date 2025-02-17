package frc.robot.subsystems.Auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import static choreo.Choreo.loadTrajectory;

public class AutoSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final AutoFactory autoFactory;

    public AutoSubsystem(DriveSubsystem drive) {
        this.drive = drive;
        this.autoFactory = new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectorySample, true, drive);
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


    public AutoRoutine MoveDepositAndClaw() {

        var trajectory = loadTrajectory(
                "BitBucketsTrajectory");

        // NEW routine
        AutoRoutine routine =
                autoFactory.newRoutine(
                        "MoveDepositAndClaw_Routine");
//
//        Command autoTrajectory =
//                autoFactory.trajectoryCmd(
//                        "BitBucketsTrajectory");
//
//        // Initialize
//        AutoRoutine routine;
//        AutoTrajectory W1toW2toDeposit =
//                routine.trajectory("W1toW2toDeposit");
//
//        AutoTrajectory W2toW3toW4toClaw =
//                routine.trajectory("W2toW3toW4toClaw");
//
//        //** AutoTrajectory W3toW4 = routine
//        // .trajectory();
//        //        AutoTrajectory W4toClaw =
//        //                routine.trajectory();
//
//        AutoTrajectory W4toW5toW6toDeposit =
//                routine.trajectory("W4toW5toW6toDeposit");
//        //AutoTrajectory W5toW6 =
//        //                routine.trajectory();
//        //        AutoTrajectory W6toDeposit =
//        //                routine.trajectory();
//
//        AutoTrajectory W6toW7toW8toClaw =
//                routine.trajectory(
//                        "W6toW7toW8toClaw");
//
//        //AutoTrajectory W7toW8 =
//        //                routine.trajectory();
//        //        AutoTrajectory W8toClaw =
//        //                routine.trajectory();
//
//        AutoTrajectory W8toW9toDeposit =
//                routine.trajectory("W8toW9toDeposit");
//
//        //AutoTrajectory W9toDeposit =
//        //                routine.trajectory();
//
//        AutoTrajectory W9toW10toClaw =
//                routine.trajectory("W9toW10toClaw");
//
//        //AutoTrajectory W10toClaw =
//        //                routine.trajectory();
//
//        AutoTrajectory W10toW11toDeposit =
//                routine.trajectory("W10toW11toDeposit");
//
//        //AutoTrajectory W11toDeposit =
//        //                routine.trajectory();
//
//
//        // TODO whatever trajectory plug in
//
//
//        rountine.active().onTrue(
//                Commands.sequence(
//                        Commands.print("Started" +
//                                " the routine!"),
//                        W1toW2toDeposit.resetOdometry(),
//                        W1toW2toDeposit.cmd(),
//                        W2toW3toW4toClaw.cmd(),
//                        W4toW5toW6toDeposit.cmd(),
//                        W6toW7toW8toClaw.cmd(),
//                        W8toW9toDeposit.cmd(),
//                        W9toW10toClaw.cmd(),
//                        W10toW11toDeposit.cmd()
//                )
//        );
//                W1toW2toDeposit.active().whileTrue(DriveSubsystem.drive());
//        W1toW2toDeposit.done().onTrue(ClawSubsystem.deposit().andThen(W2toW3toW4toClaw.cmd()));
//
//        W2toW3toW4toClaw.active().whileTrue(DriveSubsystem.drive());
//        W2toW3toW4toClaw.done().onTrue(ClawSubsystem.deposit().andThen(W4toW5toW6toDeposit.cmd()));
//
//        W4toW5toW6toDeposit.active().whileTrue(DriveSubsystem.drive());
//        W4toW5toW6toDeposit.done().onTrue(ClawSubsystem.deposit().andThen(W6toW7toW8toClaw.cmd()));
//
//        W6toW7toW8toClaw.active().whileTrue(DriveSubsystem.drive());
//        W6toW7toW8toClaw.done().onTrue(ClawSubsystem.deposit().andThen(W8toW9toDeposit.cmd()));
//
//        W8toW9toDeposit.active().whileTrue(DriveSubsystem.drive());
//        W8toW9toDeposit.done().onTrue(ClawSubsystem.deposit().andThen(W9toW10toClaw.cmd()));
//
//        W9toW10toClaw.active().whileTrue(DriveSubsystem.drive());
//        W9toW10toClaw.done().onTrue(ClawSubsystem.deposit().andThen(W10toW11toDeposit.cmd()));
//
//        W10toW11toDeposit.active().whileTrue(DriveSubsystem.drive());
//        W10toW11toDeposit.done().onTrue(ClawSubsystem.deposit());

        //TODO

//
//        routine.anyActive(W1toW2toDeposit,
//                W4toW5toW6toDeposit,
//                W8toW9toDeposit,
//                W10toW11toDeposit).whileTrue(ClawSubsystem.deposit());


        return routine;
    }
}

