package frc.robot.subsystems.Auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import static choreo.Choreo.loadTrajectory;

public class AutoSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final ClawSubsystem claw;
    private static AutoFactory autoFactory;



    public AutoSubsystem(ClawSubsystem claw,
                         DriveSubsystem drive){
        this.drive = drive;
        this.claw = claw;
        this.autoFactory = new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectorySample, true, drive);
    }



    public static Command drive() {
        return Commands.sequence(
        );
    }

    public static Command deposit() {
        return Commands.sequence(
        );
    }

    public static Command claw() {
        return Commands.sequence(
        );
    }


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------~

    public static AutoRoutine FourL4CoralBottomRoutine() {

        autoFactory
                .bind("Start4L4B", drive())

                .bind("Deposit14L4B", deposit())

                .bind("Deposit14L4B", drive())

                .bind("Intake14L4B", claw())

                .bind("Intake14L4B", drive())

                .bind("Deposit24L4B", deposit())

                .bind("Deposit24L4B", drive())

                .bind("Intake24L4B", claw())

                .bind("Intake24L4B", drive())

                .bind("Deposit34L4B", deposit())

                .bind("Deposit34L4B", drive())

                .bind("Intake34L4B", claw())

                .bind("Intake34L4B", drive())

                .bind("Deposit44L4B", deposit())

                .bind("Deposit44L4B", drive());


        var trajectory = loadTrajectory(
                "FourL4CoralBottom");

        AutoRoutine FourL4CoralBottomRoutine =
                autoFactory.newRoutine(
                        "FourL4CoralBottomRoutine");


        //Initialize
        AutoTrajectory Start4L4B =
                FourL4CoralBottomRoutine.trajectory("Start4L4B");

        AutoTrajectory Deposit14L4B =
                FourL4CoralBottomRoutine.trajectory("Start4L4B_to_Deposit14L4B");

        AutoTrajectory Intake14L4B =
                FourL4CoralBottomRoutine.trajectory("Deposit14L4B_to_Intake14L4B");

        AutoTrajectory Deposit24L4B =
                FourL4CoralBottomRoutine.trajectory("Intake14L4B_to_Deposit24L4B");

        AutoTrajectory Intake24L4B =
                FourL4CoralBottomRoutine.trajectory(
                        "Deposit24L4B_to_Intake24L4B");

        AutoTrajectory Deposit34L4B =
                FourL4CoralBottomRoutine.trajectory("Intake24L4B_to_Deposit34L4B");


        AutoTrajectory Intake34L4B =
                FourL4CoralBottomRoutine.trajectory("Deposit34L4B_to_Intake34L4B");

        AutoTrajectory Deposit44L4B =
                FourL4CoralBottomRoutine.trajectory("Intake34L4B_to_Deposit44L4B");


        FourL4CoralBottomRoutine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started" +
                                " the routine!"),
                        Start4L4B.resetOdometry()
//                        Deposit14L4B.cmd(),
//                        Intake14L4B.cmd(),
//                        Deposit24L4B.cmd(),
//                        Intake24L4B.cmd(),
//                        Deposit34L4B.cmd(),
//                        Intake34L4B.cmd(),
//                        Deposit44L4B.cmd()
                )
        );
        //prepare drive + claw subsystem
//        Start4L4B.active().whileTrue(drive());
//        Start4L4B.active().whileTrue(claw());
//        Start4L4B.active().whileTrue(deposit());
        //whileTrue is to do something together


        Start4L4B.atTime("Start4L4B").onTrue(drive());
        Start4L4B.done().onTrue(drive().andThen(Deposit14L4B.cmd()));

        Deposit14L4B.atTime("Deposit14L4B").onTrue(deposit());
        Deposit14L4B.done().onTrue(drive().andThen(Intake14L4B.cmd()));

        Intake14L4B.atTime("Intake14L4B").onTrue(claw());
        Intake14L4B.done().onTrue(drive().andThen(Deposit24L4B.cmd()));

        Deposit24L4B.atTime("Deposit24L4B").onTrue(deposit());
        Deposit24L4B.done().onTrue(drive().andThen(Intake24L4B.cmd()));

        Intake24L4B.atTime("Intake24L4B").onTrue(claw());
        Intake24L4B.done().onTrue(drive().andThen(Deposit34L4B.cmd()));

        Deposit34L4B.atTime("Deposit34L4B").onTrue(deposit());
        Deposit34L4B.done().onTrue(drive().andThen(Intake34L4B.cmd()));

        Intake34L4B.atTime("Intake34L4B").onTrue(claw());
        Intake34L4B.done().onTrue(drive().andThen(Deposit44L4B.cmd()));
//
        Deposit44L4B.atTime("Deposit44L4B").onTrue(deposit());

        //TODO


//        FourL4CoralBottomRoutine.anyActive(Start4L4BtoDeposit14L4B,
//                Intake14L4BtoDeposit24L4B,
//                Intake24L4BtoDeposit34L4B,
//                Intake34L4BtoDeposit44L4B).whileTrue(deposit());

        System.out.println(Start4L4B.getInitialPose().get());


        return FourL4CoralBottomRoutine;
    }
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

    public static AutoRoutine FourL4CoralTopRoutine() {

        var trajectory = loadTrajectory(
                "FourL4CoralTop");

        AutoRoutine FourL4CoralTopRoutine =
                autoFactory.newRoutine(
                        "FourL4CoralTopRoutine");

        // Initialize
        AutoTrajectory A1toA2toDeposit =
                FourL4CoralTopRoutine.trajectory("A1toA2toDeposit");

        AutoTrajectory A2toA3toA4toClaw =
                FourL4CoralTopRoutine.trajectory("A2toA3toA4toClaw");

        //** AutoTrajectory W3toW4 = routine
        // .trajectory();
        //        AutoTrajectory W4toClaw =
        //                routine.trajectory();

        AutoTrajectory A4toA5toA6toDeposit =
                FourL4CoralTopRoutine.trajectory("A4toA5toA6toDeposit");
        //AutoTrajectory W5toW6 =
        //                routine.trajectory();
        //        AutoTrajectory W6toDeposit =
        //                routine.trajectory();

        AutoTrajectory A6toA7toA8toClaw =
                FourL4CoralTopRoutine.trajectory(
                        "A6toA7toA8toClaw");

        //AutoTrajectory W7toW8 =
        //                routine.trajectory();
        //        AutoTrajectory W8toClaw =
        //                routine.trajectory();

        AutoTrajectory A8toA9toDeposit =
                FourL4CoralTopRoutine.trajectory("A8toA9toDeposit");

        //AutoTrajectory W9toDeposit =
        //                routine.trajectory();

        AutoTrajectory A9toA10toClaw =
                FourL4CoralTopRoutine.trajectory("A9toW10toClaw");

        //AutoTrajectory W10toClaw =
        //                routine.trajectory();

        AutoTrajectory A10toA11toDeposit =
                FourL4CoralTopRoutine.trajectory("A10toA11toDeposit");

        //AutoTrajectory W11toDeposit =
        //                routine.trajectory();


        FourL4CoralTopRoutine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started" +
                                " the routine!"),
                        A1toA2toDeposit.resetOdometry(),
                        A1toA2toDeposit.cmd(),
                        A2toA3toA4toClaw.cmd(),
                        A4toA5toA6toDeposit.cmd(),
                        A6toA7toA8toClaw.cmd(),
                        A8toA9toDeposit.cmd(),
                        A9toA10toClaw.cmd(),
                        A10toA11toDeposit.cmd()
                )
        );
        A1toA2toDeposit.active().whileTrue(drive());
        A1toA2toDeposit.done().onTrue(deposit().andThen(A2toA3toA4toClaw.cmd()));

        A2toA3toA4toClaw.active().whileTrue(drive());
        A2toA3toA4toClaw.done().onTrue(claw().andThen(A4toA5toA6toDeposit.cmd()));

        A4toA5toA6toDeposit.active().whileTrue(drive());
        A4toA5toA6toDeposit.done().onTrue(deposit().andThen(A6toA7toA8toClaw.cmd()));

        A6toA7toA8toClaw.active().whileTrue(drive());
        A6toA7toA8toClaw.done().onTrue(claw().andThen(A8toA9toDeposit.cmd()));

        A8toA9toDeposit.active().whileTrue(drive());
        A8toA9toDeposit.done().onTrue(deposit().andThen(A9toA10toClaw.cmd()));

        A9toA10toClaw.active().whileTrue(drive());
        A9toA10toClaw.done().onTrue(claw().andThen(A10toA11toDeposit.cmd()));

        A10toA11toDeposit.active().whileTrue(drive());
        A10toA11toDeposit.done().onTrue(deposit());

        //TODO


        FourL4CoralTopRoutine.anyActive(A1toA2toDeposit,
                A4toA5toA6toDeposit,
                A8toA9toDeposit,
                A10toA11toDeposit).whileTrue(deposit());

        System.out.println(A1toA2toDeposit.getInitialPose().get());


        return FourL4CoralTopRoutine;
    }

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

    public static AutoRoutine OneL4CoralMid() {

        var trajectory = loadTrajectory(
                "OneL4CoralMid");

        AutoRoutine OneL4CoralMidRoutine =
                autoFactory.newRoutine(
                        "OneL4CoralMid");

        // Initialize
        AutoTrajectory B1toB2toDeposit =
                OneL4CoralMidRoutine.trajectory("B1toB2toDeposit");


        //AutoTrajectory W11toDeposit =
        //                routine.trajectory();


        OneL4CoralMidRoutine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started" +
                                " the routine!"),
                        B1toB2toDeposit.resetOdometry(),
                        B1toB2toDeposit.cmd()

                )
        );
        B1toB2toDeposit.active().whileTrue(drive());
        B1toB2toDeposit.done().onTrue(deposit());


        //TODO


        OneL4CoralMidRoutine.anyActive(B1toB2toDeposit).whileTrue(deposit());

        System.out.println(B1toB2toDeposit.getInitialPose().get());


        return OneL4CoralMidRoutine;
    }

//    public AutoRoutine WaitTaxiBottom(){
//
//    }
//    public AutoRoutine TwoL4CoralTop(){
//
//    }
//    public AutoRoutine TwoL4CoralBottom(){
//
//    }
//    public AutoRoutine ThreeL4CoralTop(){
//
//    }
//    public AutoRoutine ThreeL4CoralBottom(){
//
//    }
//    public AutoRoutine ThreeL4CoralTop(){
//
//    }


}


//         │＼＿＿╭╭╭╭╭＿＿／ │
//        │　　　　　　　　　 　│
//        │　　　　　　　　　  　│
//        │　＞　　　　　　　●　 │
//        │≡　　╰┬┬┬╯　　≡    │
//        │　　　 ╰—╯　　　　  │
//        ╰——┬  ｏ ——————ｏ┬—╯
//        　　　│世界赛!│
//　　　        ╰┬———┬ ╯
