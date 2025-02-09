package frc.robot.subsystems.AutoSubsystem;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

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
    public static Command RaiseElevator() {
        System.out.println("RaiseElevator");
        return Commands.sequence(

        );

    }
    public static Command LowerElevator() {
        System.out.println("LowerElevator");
        return Commands.sequence(

        );
    }

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------~

    public static AutoRoutine FourL4CoralBottomRoutine() {

//        autoFactory
//                .bind("Start4L4B", drive())
//
//                .bind("Deposit14L4B", deposit())
//
//                .bind("Deposit14L4B", drive())
//
//                .bind("Intake14L4B", claw())
//
//                .bind("Intake14L4B", drive())
//
//                .bind("Deposit24L4B", deposit())
//
//                .bind("Deposit24L4B", drive())
//
//                .bind("Intake24L4B", claw())
//
//                .bind("Intake24L4B", drive())
//
//                .bind("Deposit34L4B", deposit())
//
//                .bind("Deposit34L4B", drive())
//
//                .bind("Intake34L4B", claw())
//
//                .bind("Intake34L4B", drive())
//
//                .bind("Deposit44L4B", deposit())
//
//                .bind("Deposit44L4B", drive());

//        var trajectory = loadTrajectory(
//                "FourL4CoralBottom");

        AutoRoutine FourL4CoralBottomRoutine =
                autoFactory.newRoutine(
                        "FourL4CoralBottomRoutine");


        //Initialize
//1
        AutoTrajectory StarttoR8 =
                FourL4CoralBottomRoutine.trajectory("StarttoR8");
//2
        AutoTrajectory R8toSource =
                FourL4CoralBottomRoutine.trajectory("R8toSource");
//3
        AutoTrajectory SourcetoR7 =
                FourL4CoralBottomRoutine.trajectory("SourcetoR7");
//4
        AutoTrajectory R7toSource =
                FourL4CoralBottomRoutine.trajectory("R7toSource");
//5
        AutoTrajectory SourcetoR6 =
                FourL4CoralBottomRoutine.trajectory("SourcetoR6");
//6
        AutoTrajectory R6toSource =
                FourL4CoralBottomRoutine.trajectory("R6toSource");
//7
        AutoTrajectory SourcetoR5 =
                FourL4CoralBottomRoutine.trajectory("SourcetoR5");
//8
        AutoTrajectory R5toSource =
                FourL4CoralBottomRoutine.trajectory("R5toSource");


        FourL4CoralBottomRoutine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started" +
                                " the routine!"),
                        StarttoR8.resetOdometry(),
                        R8toSource.cmd(),
                        SourcetoR7.cmd(),
                        R7toSource.cmd(),
                        SourcetoR6.cmd(),
                        R6toSource.cmd(),
                        SourcetoR5.cmd(),
                        R5toSource.cmd()
                )
        );
        //prepare drive + claw subsystem
//        Start4L4B.active().whileTrue(drive());
//        Start4L4B.active().whileTrue(claw());
//        Start4L4B.active().whileTrue(deposit());
        //whileTrue is to do something together


        //4L4BottomCoral Chrono:
//1 StarttoR8
//2 R8toSource
//3 SourcetoR7
//4 R7toSource
//5 SourcetoR6
//6 R6toSource
//7 SourcetoR5
//8 R5toSource


        StarttoR8.atTime("StarttoR8").onTrue(drive());
        StarttoR8.done().onTrue(drive().andThen(R8toSource.cmd(), LowerElevator()));

        R8toSource.atTime("R8toSource").onTrue(deposit());
        R8toSource.done().onTrue(drive().andThen(SourcetoR7.cmd(), RaiseElevator()));


        SourcetoR7.atTime("SourcetoR7").onTrue(claw());
        SourcetoR7.done().onTrue(drive().andThen(R7toSource.cmd(), LowerElevator()));

        R7toSource.atTime("R7toSource").onTrue(deposit());
        R7toSource.done().onTrue(drive().andThen(SourcetoR6.cmd(), RaiseElevator()));

        SourcetoR6.atTime("SourcetoR6").onTrue(claw());
        SourcetoR6.done().onTrue(drive().andThen(R6toSource.cmd(), LowerElevator()));

        R6toSource.atTime("R6toSource").onTrue(deposit());
        R6toSource.done().onTrue(drive().andThen(SourcetoR5.cmd(), RaiseElevator()));

        SourcetoR5.atTime("SourcetoR5").onTrue(claw());
        SourcetoR5.done().onTrue(drive().andThen(R5toSource.cmd(), LowerElevator()));

        R5toSource.atTime("R5toSource").onTrue(deposit());
        SourcetoR5.done();

//        Deposit14L4B.atTime("Deposit14L4B").onTrue(deposit());
//        Deposit14L4B.done().onTrue(drive().andThen(Intake14L4B.cmd()));
//
//        Intake14L4B.atTime("Intake14L4B").onTrue(claw());
//        Intake14L4B.done().onTrue(drive().andThen(Deposit24L4B.cmd()));
//
//        Deposit24L4B.atTime("Deposit24L4B").onTrue(deposit());
//        Deposit24L4B.done().onTrue(drive().andThen(Intake24L4B.cmd()));
//
//        Intake24L4B.atTime("Intake24L4B").onTrue(claw());
//        Intake24L4B.done().onTrue(drive().andThen(Deposit34L4B.cmd()));
//
//        Deposit34L4B.atTime("Deposit34L4B").onTrue(deposit());
//        Deposit34L4B.done().onTrue(drive().andThen(Intake34L4B.cmd()));
//
//        Intake34L4B.atTime("Intake34L4B").onTrue(claw());
//        Intake34L4B.done().onTrue(drive().andThen(Deposit44L4B.cmd()));
////
//        Deposit44L4B.atTime("Deposit44L4B").onTrue(deposit());

        //TODO


//        FourL4CoralBottomRoutine.anyActive(Start4L4BtoDeposit14L4B,
//                Intake14L4BtoDeposit24L4B,
//                Intake24L4BtoDeposit34L4B,
//                Intake34L4BtoDeposit44L4B).whileTrue(deposit());

        System.out.println(StarttoR8.getInitialPose().get());


        return FourL4CoralBottomRoutine;
    }
    //4L4TopCoral Chrono:
//1 StarttoR11
//2 R11toSource
//3 SourcetoR12
//4 R12toSource
//5 SourcetoR1
//6 R1toSource
//7 SourcetoR2
//8 R2toSource
//    public static AutoRoutine FourL4CoralTopRoutine() {
//
//        var trajectory = loadTrajectory(
//                "FourL4CoralTop");
//
//        AutoRoutine FourL4CoralTopRoutine =
//                autoFactory.newRoutine(
//                        "FourL4CoralTopRoutine");
//
//        // Initialize
//        AutoTrajectory A1toA2toDeposit =
//                FourL4CoralTopRoutine.trajectory("A1toA2toDeposit");
//
//        AutoTrajectory A2toA3toA4toClaw =
//                FourL4CoralTopRoutine.trajectory("A2toA3toA4toClaw");
//
//        //** AutoTrajectory W3toW4 = routine
//        // .trajectory();
//        //        AutoTrajectory W4toClaw =
//        //                routine.trajectory();
//
//        AutoTrajectory A4toA5toA6toDeposit =
//                FourL4CoralTopRoutine.trajectory("A4toA5toA6toDeposit");
//        //AutoTrajectory W5toW6 =
//        //                routine.trajectory();
//        //        AutoTrajectory W6toDeposit =
//        //                routine.trajectory();
//
//        AutoTrajectory A6toA7toA8toClaw =
//                FourL4CoralTopRoutine.trajectory(
//                        "A6toA7toA8toClaw");
//
//        //AutoTrajectory W7toW8 =
//        //                routine.trajectory();
//        //        AutoTrajectory W8toClaw =
//        //                routine.trajectory();
//
//        AutoTrajectory A8toA9toDeposit =
//                FourL4CoralTopRoutine.trajectory("A8toA9toDeposit");
//
//        //AutoTrajectory W9toDeposit =
//        //                routine.trajectory();
//
//        AutoTrajectory A9toA10toClaw =
//                FourL4CoralTopRoutine.trajectory("A9toW10toClaw");
//
//        //AutoTrajectory W10toClaw =
//        //                routine.trajectory();
//
//        AutoTrajectory A10toA11toDeposit =
//                FourL4CoralTopRoutine.trajectory("A10toA11toDeposit");
//
//        //AutoTrajectory W11toDeposit =
//        //                routine.trajectory();
//
//
//        FourL4CoralTopRoutine.active().onTrue(
//                Commands.sequence(
//                        Commands.print("Started" +
//                                " the routine!"),
//                        A1toA2toDeposit.resetOdometry(),
//                        A1toA2toDeposit.cmd(),
//                        A2toA3toA4toClaw.cmd(),
//                        A4toA5toA6toDeposit.cmd(),
//                        A6toA7toA8toClaw.cmd(),
//                        A8toA9toDeposit.cmd(),
//                        A9toA10toClaw.cmd(),
//                        A10toA11toDeposit.cmd()
//                )
//        );
//        A1toA2toDeposit.active().whileTrue(drive());
//        A1toA2toDeposit.done().onTrue(deposit().andThen(A2toA3toA4toClaw.cmd()));
//
//        A2toA3toA4toClaw.active().whileTrue(drive());
//        A2toA3toA4toClaw.done().onTrue(claw().andThen(A4toA5toA6toDeposit.cmd()));
//
//        A4toA5toA6toDeposit.active().whileTrue(drive());
//        A4toA5toA6toDeposit.done().onTrue(deposit().andThen(A6toA7toA8toClaw.cmd()));
//
//        A6toA7toA8toClaw.active().whileTrue(drive());
//        A6toA7toA8toClaw.done().onTrue(claw().andThen(A8toA9toDeposit.cmd()));
//
//        A8toA9toDeposit.active().whileTrue(drive());
//        A8toA9toDeposit.done().onTrue(deposit().andThen(A9toA10toClaw.cmd()));
//
//        A9toA10toClaw.active().whileTrue(drive());
//        A9toA10toClaw.done().onTrue(claw().andThen(A10toA11toDeposit.cmd()));
//
//        A10toA11toDeposit.active().whileTrue(drive());
//        A10toA11toDeposit.done().onTrue(deposit());
//
//        //TODO
//
//
//        FourL4CoralTopRoutine.anyActive(A1toA2toDeposit,
//                A4toA5toA6toDeposit,
//                A8toA9toDeposit,
//                A10toA11toDeposit).whileTrue(deposit());
//
//        System.out.println(A1toA2toDeposit.getInitialPose().get());
//
//
//        return FourL4CoralTopRoutine;
//    }
//
////----------------------------------------------------------------------------
////----------------------------------------------------------------------------
//
//    public static AutoRoutine OneL4CoralMid() {
//
//        var trajectory = loadTrajectory(
//                "OneL4CoralMid");
//
//        AutoRoutine OneL4CoralMidRoutine =
//                autoFactory.newRoutine(
//                        "OneL4CoralMid");
//
//        // Initialize
//        AutoTrajectory B1toB2toDeposit =
//                OneL4CoralMidRoutine.trajectory("B1toB2toDeposit");
//
//
//        //AutoTrajectory W11toDeposit =
//        //                routine.trajectory();
//
//
//        OneL4CoralMidRoutine.active().onTrue(
//                Commands.sequence(
//                        Commands.print("Started" +
//                                " the routine!"),
//                        B1toB2toDeposit.resetOdometry(),
//                        B1toB2toDeposit.cmd()
//
//                )
//        );
//        B1toB2toDeposit.active().whileTrue(drive());
//        B1toB2toDeposit.done().onTrue(deposit());
//
//
//        //TODO
//
//
//        OneL4CoralMidRoutine.anyActive(B1toB2toDeposit).whileTrue(deposit());
//
//        System.out.println(B1toB2toDeposit.getInitialPose().get());
//
//
//        return OneL4CoralMidRoutine;
//    }
//
////    public AutoRoutine WaitTaxiBottom(){
////
////    }
////    public AutoRoutine TwoL4CoralTop(){
////
////    }
////    public AutoRoutine TwoL4CoralBottom(){
////
////    }
////    public AutoRoutine ThreeL4CoralTop(){
////
////    }
////    public AutoRoutine ThreeL4CoralBottom(){
////
////    }
////    public AutoRoutine ThreeL4CoralTop(){
////
////    }
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
