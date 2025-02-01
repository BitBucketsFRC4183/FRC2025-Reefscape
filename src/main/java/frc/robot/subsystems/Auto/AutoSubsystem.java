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

        var trajectory = loadTrajectory(
                "FourL4CoralBottom");

        AutoRoutine FourL4CoralBottomRoutine =
                autoFactory.newRoutine(
                        "FourL4CoralBottomRoutine");

        // Initialize
        AutoTrajectory W1toW2toDeposit =
                FourL4CoralBottomRoutine.trajectory("W1toW2toDeposit");

        AutoTrajectory W2toW3toW4toClaw =
                FourL4CoralBottomRoutine.trajectory("W2toW3toW4toClaw");

        //** AutoTrajectory W3toW4 = routine
        // .trajectory();
        //        AutoTrajectory W4toClaw =
        //                routine.trajectory();

        AutoTrajectory W4toW5toW6toDeposit =
                FourL4CoralBottomRoutine.trajectory("W4toW5toW6toDeposit");
        //AutoTrajectory W5toW6 =
        //                routine.trajectory();
        //        AutoTrajectory W6toDeposit =
        //                routine.trajectory();

        AutoTrajectory W6toW7toW8toClaw =
                FourL4CoralBottomRoutine.trajectory(
                        "W6toW7toW8toClaw");

        //AutoTrajectory W7toW8 =
        //                routine.trajectory();
        //        AutoTrajectory W8toClaw =
        //                routine.trajectory();

        AutoTrajectory W8toW9toDeposit =
                FourL4CoralBottomRoutine.trajectory("W8toW9toDeposit");

        //AutoTrajectory W9toDeposit =
        //                routine.trajectory();

        AutoTrajectory W9toW10toClaw =
                FourL4CoralBottomRoutine.trajectory("W9toW10toClaw");

        //AutoTrajectory W10toClaw =
        //                routine.trajectory();

        AutoTrajectory W10toW11toDeposit =
                FourL4CoralBottomRoutine.trajectory("W10toW11toDeposit");

        //AutoTrajectory W11toDeposit =
        //                routine.trajectory();

        FourL4CoralBottomRoutine.active().onTrue(
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


        FourL4CoralBottomRoutine.anyActive(W1toW2toDeposit,
                W4toW5toW6toDeposit,
                W8toW9toDeposit,
                W10toW11toDeposit).whileTrue(deposit());

        System.out.println(W1toW2toDeposit.getInitialPose().get());


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


//         │＼＿＿╭╭╭╭╭＿＿／│
//        │　　　　　　　　　　│
//        │　　　　　　　　　　│
//        │　＞　　　　　　　●　│
//        │≡　　╰┬┬┬╯　　≡   │
//        │　　　 ╰—╯　　　　 │
//        ╰——┬  ｏ ——————ｏ┬—╯
//        　　　│世界赛!│
//　　　        ╰┬———┬ ╯

