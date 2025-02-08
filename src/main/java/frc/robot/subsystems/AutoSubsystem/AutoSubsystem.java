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
                         DriveSubsystem drive) {
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

        System.out.println(StarttoR8.getInitialPose().get());


        return FourL4CoralBottomRoutine;
    }
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
