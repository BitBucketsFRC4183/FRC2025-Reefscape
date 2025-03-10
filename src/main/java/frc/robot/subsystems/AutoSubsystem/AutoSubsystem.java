package frc.robot.subsystems.AutoSubsystem;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ArmElevatorToOrigin;
import frc.robot.commands.ArmElevatorToSetpoint;
import frc.robot.commands.DriveCommands.RobotRelativeDriveCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;


public class AutoSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private static AutoFactory autoFactory;


    public AutoSubsystem(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.drive = drive;
        this.elevator = elevator;
        this.arm = arm;
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
//----------------------------------------------------------------------------

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
                                " FourL4CoralBottomRoutine" +
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

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

    public static AutoRoutine FourL4CoralTopRoutine() {

        //        var trajectory = loadTrajectory(
        //                "FourL4CoralBottom");

        AutoRoutine FourL4CoralTopRoutine =
                autoFactory.newRoutine(
                        "FourL4CoralTopRoutine");

        //Initialize
        //1
        AutoTrajectory StarttoR11 =
                FourL4CoralTopRoutine.trajectory("StarttoR11");
        //2
        AutoTrajectory R11toSource =
                FourL4CoralTopRoutine.trajectory("R11toSource");
        //3
        AutoTrajectory SourcetoR12 =
                FourL4CoralTopRoutine.trajectory("SourcetoR12");
        //4
        AutoTrajectory R12toSource =
                FourL4CoralTopRoutine.trajectory("R12toSource");
        //5
        AutoTrajectory SourcetoR1 =
                FourL4CoralTopRoutine.trajectory("SourcetoR1");
        //6
        AutoTrajectory R1toSource =
                FourL4CoralTopRoutine.trajectory("R1toSource");
        //7
        AutoTrajectory SourcetoR2 =
                FourL4CoralTopRoutine.trajectory("SourcetoR2");
        //8
        AutoTrajectory R2toSource =
                FourL4CoralTopRoutine.trajectory("R2toSource");

        FourL4CoralTopRoutine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started" +
                                "FourL4CoralTopRoutine " +
                                " the routine!"),
                        StarttoR11.resetOdometry(),
                        R11toSource.cmd(),
                        SourcetoR12.cmd(),
                        R12toSource.cmd(),
                        SourcetoR1.cmd(),
                        R1toSource.cmd(),
                        SourcetoR2.cmd(),
                        R2toSource.cmd()
                )
        );


        StarttoR11.atTime("StarttoR11").onTrue(drive());
        StarttoR11.done().onTrue(drive().andThen(R11toSource.cmd(), LowerElevator()));

        R11toSource.atTime("R11toSource").onTrue(deposit());
        R11toSource.done().onTrue(drive().andThen(SourcetoR12.cmd(), RaiseElevator()));


        SourcetoR12.atTime("SourcetoR12").onTrue(claw());
        SourcetoR12.done().onTrue(drive().andThen(R12toSource.cmd(), LowerElevator()));

        R12toSource.atTime("R12toSource").onTrue(deposit());
        R12toSource.done().onTrue(drive().andThen(SourcetoR1.cmd(), RaiseElevator()));

        SourcetoR1.atTime("SourcetoR1").onTrue(claw());
        SourcetoR1.done().onTrue(drive().andThen(R1toSource.cmd(), LowerElevator()));

        R1toSource.atTime("R1toSource").onTrue(deposit());
        R1toSource.done().onTrue(drive().andThen(SourcetoR2.cmd(), RaiseElevator()));

        SourcetoR2.atTime("SourcetoR2").onTrue(claw());
        SourcetoR2.done().onTrue(drive().andThen(R2toSource.cmd(), LowerElevator()));

        R2toSource.atTime("R2toSource").onTrue(deposit());
        R2toSource.done();

        System.out.println(StarttoR11.getInitialPose().get());


        return FourL4CoralTopRoutine;
    }

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------


    public static AutoRoutine ThreeL4CoralBottomRoutine() {

        AutoRoutine ThreeL4CoralBottomRoutine =
                autoFactory.newRoutine(
                        "ThreeL4CoralBottomRoutine");

        AutoTrajectory StarttoR8 =
                ThreeL4CoralBottomRoutine.trajectory("StarttoR8");
//2
        AutoTrajectory R8toSource =
                ThreeL4CoralBottomRoutine.trajectory("R8toSource");
//3
        AutoTrajectory SourcetoR7 =
                ThreeL4CoralBottomRoutine.trajectory("SourcetoR7");
//4
        AutoTrajectory R7toSource =
                ThreeL4CoralBottomRoutine.trajectory("R7toSource");
//5
        AutoTrajectory SourcetoR6 =
                ThreeL4CoralBottomRoutine.trajectory("SourcetoR6");

        ThreeL4CoralBottomRoutine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started" +
                                "ThreeL4CoralBottomRoutine" +
                                " the routine!"),
                        StarttoR8.resetOdometry(),
                        R8toSource.cmd(),
                        SourcetoR7.cmd(),
                        R7toSource.cmd(),
                        SourcetoR6.cmd()
                )
        );


        StarttoR8.atTime("StarttoR8").onTrue(drive());
        StarttoR8.done().onTrue(drive().andThen(R8toSource.cmd(), LowerElevator()));

        R8toSource.atTime("R8toSource").onTrue(deposit());
        R8toSource.done().onTrue(drive().andThen(SourcetoR7.cmd(), RaiseElevator()));


        SourcetoR7.atTime("SourcetoR7").onTrue(claw());
        SourcetoR7.done().onTrue(drive().andThen(R7toSource.cmd(), LowerElevator()));

        R7toSource.atTime("R7toSource").onTrue(deposit());
        R7toSource.done().onTrue(drive().andThen(SourcetoR6.cmd(), RaiseElevator()));

        SourcetoR6.atTime("SourcetoR6").onTrue(claw());
        SourcetoR6.done();

        System.out.println(StarttoR8.getInitialPose().get());


        return ThreeL4CoralBottomRoutine;
    }

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

    public static AutoRoutine ThreeL4CoralTopRoutine() {

        //        var trajectory = loadTrajectory(
        //                "FourL4CoralBottom");

        AutoRoutine ThreeL4CoralTopRoutine =
                autoFactory.newRoutine(
                        "ThreeL4CoralTopRoutine");

        //Initialize
        //1
        AutoTrajectory StarttoR11 =
                ThreeL4CoralTopRoutine.trajectory("StarttoR11");
        //2
        AutoTrajectory R11toSource =
                ThreeL4CoralTopRoutine.trajectory("R11toSource");
        //3
        AutoTrajectory SourcetoR12 =
                ThreeL4CoralTopRoutine.trajectory("SourcetoR12");
        //4
        AutoTrajectory R12toSource =
                ThreeL4CoralTopRoutine.trajectory("R12toSource");
        //5
        AutoTrajectory SourcetoR1 =
                ThreeL4CoralTopRoutine.trajectory("SourcetoR1");

        ThreeL4CoralTopRoutine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started" +
                                "ThreeL4CoralTopRoutine" +
                                " the routine!"),
                        StarttoR11.resetOdometry(),
                        R11toSource.cmd(),
                        SourcetoR12.cmd(),
                        R12toSource.cmd(),
                        SourcetoR1.cmd()

                )
        );


        StarttoR11.atTime("StarttoR11").onTrue(drive());
        StarttoR11.done().onTrue(drive().andThen(R11toSource.cmd(), LowerElevator()));

        R11toSource.atTime("R11toSource").onTrue(deposit());
        R11toSource.done().onTrue(drive().andThen(SourcetoR12.cmd(), RaiseElevator()));


        SourcetoR12.atTime("SourcetoR12").onTrue(claw());
        SourcetoR12.done().onTrue(drive().andThen(R12toSource.cmd(), LowerElevator()));

        R12toSource.atTime("R12toSource").onTrue(deposit());
        R12toSource.done().onTrue(drive().andThen(SourcetoR1.cmd(), RaiseElevator()));

        SourcetoR1.atTime("SourcetoR1").onTrue(claw());
        SourcetoR1.done();

        System.out.println(StarttoR11.getInitialPose().get());


        return ThreeL4CoralTopRoutine;
    }

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
public static AutoRoutine OneL4CoralMidRoutine() {

    //        var trajectory = loadTrajectory(
    //                "FourL4CoralBottom");

    AutoRoutine OneL4CoralMidRoutine =
            autoFactory.newRoutine(
                    "OneL4CoralMidRoutine");

    //Initialize
    //1
    AutoTrajectory StarttoR9 =
            OneL4CoralMidRoutine.trajectory("StarttoR9");
    //2
    AutoTrajectory R9toStart =
            OneL4CoralMidRoutine.trajectory("R9toStart");


    OneL4CoralMidRoutine.active().onTrue(
            Commands.sequence(
                    Commands.print("Started" +
                            "OneL4CoralMidRoutine" +
                            " the routine!"),
                    StarttoR9.resetOdometry(),
                    R9toStart.cmd()

            )
    );


    StarttoR9.atTime("StarttoR9").onTrue(drive());
    StarttoR9.done().onTrue(drive().andThen(R9toStart.cmd(), LowerElevator()));

    R9toStart.atTime("R9toStart").onTrue(deposit());
    R9toStart.done();



    System.out.println(StarttoR9.getInitialPose().get());


    return OneL4CoralMidRoutine;


    }


    public Command OneL3Score(ElevatorSubsystem elevator, ArmSubsystem arm) {
        return Commands.sequence(
                Commands.deadline(Commands.waitSeconds(4), new RobotRelativeDriveCommand(drive, () -> 0.2, () -> 0, () -> 0)),
                Commands.waitSeconds(1),
                Commands.deadline(Commands.waitSeconds(0.2), new RobotRelativeDriveCommand(drive, () -> -0.2, () -> 0, () -> 0)),
                Commands.deadline(Commands.waitSeconds(3), new ArmElevatorToSetpoint(elevator, arm, ElevatorConstants.L3 + 0.005, ArmConstants.armL4Angle + Units.degreesToRadians(1))),
                Commands.deadline(Commands.waitSeconds(0.05), new RobotRelativeDriveCommand(drive, () -> 0.1, () -> 0, () -> 0)),
                Commands.parallel(
                        new ArmElevatorToOrigin(elevator, arm),
                        Commands.deadline(Commands.waitSeconds(0.2), new RobotRelativeDriveCommand(drive, () -> -0.1, () -> 0, () -> 0))
                        )


                );
    }

    public Command TaxiBack() {
        return Commands.sequence(
                Commands.waitSeconds(0),
                Commands.deadline(
                        Commands.waitSeconds(3),
                        new RobotRelativeDriveCommand(drive, () -> -1, () -> 0, () -> 0)
                )
        );
    };

    public Command TaxiForward() {
        return Commands.sequence(
                Commands.waitSeconds(0),
                Commands.deadline(
                        Commands.waitSeconds(3),
                        new RobotRelativeDriveCommand(drive, () -> 1, () -> 0, () -> 0)
                )
        );
    };
}



//         │＼＿＿╭╭╭╭╭＿＿│
//        │　　　　　　　　　 　│
//        │　　　　　　　　　  　│
//        │　＞　　　　　　　│
//        │≡　  ╰┬┬┬╯　　≡    │
//        │　　 　╰—╯  　　　  │
//        ╰——┬   ——————┬—╯
//        　　　│世界赛!│
//　　　        ╰┬———┬ ╯
