package frc.robot.subsystems.Auto;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeManagementSubsystem.AlgaeManagementSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import static choreo.Choreo.loadTrajectory;

public class AutoSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final ClawSubsystem claw;
    private final AutoFactory autoFactory;



    public AutoSubsystem(ClawSubsystem claw,
                         DriveSubsystem drive){
        this.drive = drive;
        this.claw = claw;
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

    public AutoRoutine WaitTaxiBottom(){

    }
    public AutoRoutine TwoL4CoralTop(){

    }
    public AutoRoutine TwoL4CoralBottom(){

    }
    public AutoRoutine ThreeL4CoralTop(){

    }
    public AutoRoutine ThreeL4CoralBottom(){

    }
    public AutoRoutine ThreeL4CoralTop(){

    }
    public AutoRoutine FourL4CoralTop(){
        var trajectory = loadTrajectory(
                "FourL4CoralTop");

        AutoRoutine routine =
                autoFactory.newRoutine(
                        "FourL4CoralTop");

        // Initialize
        AutoTrajectory W1toW2toDeposit =
                routine.trajectory("W1toW2toDeposit");

        AutoTrajectory W2toW3toW4toClaw =
                routine.trajectory("W2toW3toW4toClaw");

        //** AutoTrajectory W3toW4 = routine
        // .trajectory();
        //        AutoTrajectory W4toClaw =
        //                routine.trajectory();

        AutoTrajectory W4toW5toW6toDeposit =
                routine.trajectory("W4toW5toW6toDeposit");
        //AutoTrajectory W5toW6 =
        //                routine.trajectory();
        //        AutoTrajectory W6toDeposit =
        //                routine.trajectory();

        AutoTrajectory W6toW7toW8toClaw =
                routine.trajectory(
                        "W6toW7toW8toClaw");

        //AutoTrajectory W7toW8 =
        //                routine.trajectory();
        //        AutoTrajectory W8toClaw =
        //                routine.trajectory();

        AutoTrajectory W8toW9toDeposit =
                routine.trajectory("W8toW9toDeposit");

        //AutoTrajectory W9toDeposit =
        //                routine.trajectory();

        AutoTrajectory W9toW10toClaw =
                routine.trajectory("W9toW10toClaw");

        //AutoTrajectory W10toClaw =
        //                routine.trajectory();

        AutoTrajectory W10toW11toDeposit =
                routine.trajectory("W10toW11toDeposit");

        //AutoTrajectory W11toDeposit =
        //                routine.trajectory();
//
//
//
//
        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started" +
                                " the routine!"),
                        W1toW2toDeposit.resetOdometry(),
                        W1toW2toDeposit.cmd(),
                        W2toW3toW4toClaw.cmd(),
                        W4toW5toW6toDeposit.cmd(),
                        W6toW7toW8toClaw.cmd(),
                        W8toW9toDeposit.cmd(),
                        W9toW10toClaw.cmd(),
                        W10toW11toDeposit.cmd()
                )
        );
        W1toW2toDeposit.active().whileTrue(drive());
        W1toW2toDeposit.done().onTrue(deposit().andThen(W2toW3toW4toClaw.cmd()));

        W2toW3toW4toClaw.active().whileTrue(drive());
        W2toW3toW4toClaw.done().onTrue(claw().andThen(W4toW5toW6toDeposit.cmd()));

        W4toW5toW6toDeposit.active().whileTrue(drive());
        W4toW5toW6toDeposit.done().onTrue(deposit().andThen(W6toW7toW8toClaw.cmd()));

        W6toW7toW8toClaw.active().whileTrue(drive());
        W6toW7toW8toClaw.done().onTrue(claw().andThen(W8toW9toDeposit.cmd()));

        W8toW9toDeposit.active().whileTrue(drive());
        W8toW9toDeposit.done().onTrue(deposit().andThen(W9toW10toClaw.cmd()));

        W9toW10toClaw.active().whileTrue(drive());
        W9toW10toClaw.done().onTrue(claw().andThen(W10toW11toDeposit.cmd()));

        W10toW11toDeposit.active().whileTrue(drive());
        W10toW11toDeposit.done().onTrue(deposit());

        //TODO


        routine.anyActive(W1toW2toDeposit,
                W4toW5toW6toDeposit,
                W8toW9toDeposit,
                W10toW11toDeposit).whileTrue(deposit());


        return routine;
    }

    public AutoRoutine FourL4CoralBottom() {

        var trajectory = loadTrajectory(
                "FourL4CoralBottom");

        AutoRoutine routine =
                autoFactory.newRoutine(
                        "FourL4CoralBottom");

        // Initialize
        AutoTrajectory W1toW2toDeposit =
                routine.trajectory("W1toW2toDeposit");

        AutoTrajectory W2toW3toW4toClaw =
                routine.trajectory("W2toW3toW4toClaw");

        //** AutoTrajectory W3toW4 = routine
        // .trajectory();
        //        AutoTrajectory W4toClaw =
        //                routine.trajectory();

        AutoTrajectory W4toW5toW6toDeposit =
                routine.trajectory("W4toW5toW6toDeposit");
        //AutoTrajectory W5toW6 =
        //                routine.trajectory();
        //        AutoTrajectory W6toDeposit =
        //                routine.trajectory();

        AutoTrajectory W6toW7toW8toClaw =
                routine.trajectory(
                        "W6toW7toW8toClaw");

        //AutoTrajectory W7toW8 =
        //                routine.trajectory();
        //        AutoTrajectory W8toClaw =
        //                routine.trajectory();

        AutoTrajectory W8toW9toDeposit =
                routine.trajectory("W8toW9toDeposit");

        //AutoTrajectory W9toDeposit =
        //                routine.trajectory();

        AutoTrajectory W9toW10toClaw =
                routine.trajectory("W9toW10toClaw");

        //AutoTrajectory W10toClaw =
        //                routine.trajectory();

        AutoTrajectory W10toW11toDeposit =
                routine.trajectory("W10toW11toDeposit");

        //AutoTrajectory W11toDeposit =
        //                routine.trajectory();
//
//
//
//
        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started" +
                                " the routine!"),
                        W1toW2toDeposit.resetOdometry(),
                        W1toW2toDeposit.cmd(),
                        W2toW3toW4toClaw.cmd(),
                        W4toW5toW6toDeposit.cmd(),
                        W6toW7toW8toClaw.cmd(),
                        W8toW9toDeposit.cmd(),
                        W9toW10toClaw.cmd(),
                        W10toW11toDeposit.cmd()
                )
        );
                W1toW2toDeposit.active().whileTrue(drive());
        W1toW2toDeposit.done().onTrue(deposit().andThen(W2toW3toW4toClaw.cmd()));

        W2toW3toW4toClaw.active().whileTrue(drive());
        W2toW3toW4toClaw.done().onTrue(claw().andThen(W4toW5toW6toDeposit.cmd()));

        W4toW5toW6toDeposit.active().whileTrue(drive());
        W4toW5toW6toDeposit.done().onTrue(deposit().andThen(W6toW7toW8toClaw.cmd()));

        W6toW7toW8toClaw.active().whileTrue(drive());
        W6toW7toW8toClaw.done().onTrue(claw().andThen(W8toW9toDeposit.cmd()));

        W8toW9toDeposit.active().whileTrue(drive());
        W8toW9toDeposit.done().onTrue(deposit().andThen(W9toW10toClaw.cmd()));

        W9toW10toClaw.active().whileTrue(drive());
        W9toW10toClaw.done().onTrue(claw().andThen(W10toW11toDeposit.cmd()));

        W10toW11toDeposit.active().whileTrue(drive());
        W10toW11toDeposit.done().onTrue(deposit());

        //TODO


        routine.anyActive(W1toW2toDeposit,
                W4toW5toW6toDeposit,
                W8toW9toDeposit,
                W10toW11toDeposit).whileTrue(deposit());


        return routine;
    }

}


//         │＼＿＿╭╭╭╭╭＿＿／│
//        │　　　　　　　　　　│
//        │　　　　　　　　　　│
//        │　＞　　　　　　　●　│
//        │≡　　╰┬┬┬╯　　≡   │
//        │　　　 ╰—╯　　　　 │
//        ╰——┬  ｏ ——————ｏ┬—╯
//        　　　│世界赛!│
//　　　        ╰┬———┬ ╯

