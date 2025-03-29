package frc.robot.subsystems.AutoSubsystem;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.DriveCommands.RobotRelativeDriveCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;


public class AutoSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final ClawSubsystem claw;
    private final AutoFactory autoFactory;

    private static Choreo.TrajectoryLogger<SwerveSample> trajectoryLogger() {
        return (swerveSampleTrajectory, aBoolean) -> {
            Logger.recordOutput(
                      "Odometry/Trajectory", swerveSampleTrajectory.getPoses()
            );
            Logger.recordOutput("Odometry/TrajectoryGoal", swerveSampleTrajectory.getFinalPose(false).get());
        };
    }
    public AutoSubsystem(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, ClawSubsystem claw) {
        this.drive = drive;
        this.elevator = elevator;
        this.arm = arm;
        this.claw = claw;
        this.autoFactory = new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectorySample, false, drive, trajectoryLogger());

    }

    public Command raiseArmElevatorToL4() {
        return deadline(waitSeconds(2.4), new ArmElevatorToSetpointAuto(elevator, arm, ElevatorConstants.L4, ArmConstants.armL4Angle));
    }
    public Command lowerArmElevatorToOrigin() {
        return deadline(waitSeconds(1.5), new ArmElevatorToOrigin(elevator,arm));
    }
    public Command stop() {
        return Commands.runOnce(drive::stop);
    }
    public Command score() {
        return Commands.parallel(Commands.deadline(waitSeconds(0.8), new ArmElevatorToSetpoint(elevator, arm, ElevatorConstants.L4 - 0.25, Units.degreesToRadians(-20)), waitSeconds(0.1)));
    }
    public Command coralIntake() {
        return deadline(waitSeconds(0.25), new CoralIntake(elevator, arm, claw));
    }


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

    public AutoRoutine ThreeL4CoralTopRoutine() {

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
        AutoTrajectory R11Forward =
                ThreeL4CoralTopRoutine.trajectory("R11Forward");
        //3
        AutoTrajectory R11toSource =
                ThreeL4CoralTopRoutine.trajectory("R11toSource");
        //4
        AutoTrajectory SourcetoR12 =
                ThreeL4CoralTopRoutine.trajectory("SourcetoR12");
        //5
        AutoTrajectory R12Forward =
                ThreeL4CoralTopRoutine.trajectory("R12Forward");
       //6
        AutoTrajectory R12toSource =
                ThreeL4CoralTopRoutine.trajectory("R12toSource");
        //7
        AutoTrajectory SourcetoR1 =
                ThreeL4CoralTopRoutine.trajectory("SourcetoR1");
        //8
        AutoTrajectory R1Forward =
                ThreeL4CoralTopRoutine.trajectory("R1Forward");


        ThreeL4CoralTopRoutine.active().onTrue(
                sequence(
                        Commands.print("Started" +
                                "ThreeL4CoralTopRoutine" +
                                " the routine!"),
                        StarttoR11.resetOdometry(),
                        StarttoR11.cmd()

                )
        );

        StarttoR11.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), R11Forward.cmd()));

        R11Forward.done().onTrue(sequence(stop(), score(), parallel(R11toSource.cmd(), lowerArmElevatorToOrigin())));

        R11toSource.done().onTrue(sequence(stop(), waitSeconds(1), coralIntake(), SourcetoR12.cmd()));

        SourcetoR12.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), R12Forward.cmd()));

        R12Forward.done().onTrue(sequence(stop(), score(), parallel(R12toSource.cmd(), lowerArmElevatorToOrigin())));

        R12toSource.done().onTrue(sequence(stop(), waitSeconds(1), coralIntake(), SourcetoR1.cmd()));

        SourcetoR1.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), R1Forward.cmd()));

        R1Forward.done().onTrue(sequence(stop(), score(), lowerArmElevatorToOrigin()));





        

        return ThreeL4CoralTopRoutine;
    }

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
public AutoRoutine OneL4CoralMidRoutine() {

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
    AutoTrajectory R9Backup =
            OneL4CoralMidRoutine.trajectory("R9Backup");
    //3
    AutoTrajectory R9Forward =
            OneL4CoralMidRoutine.trajectory("R9Forward");
    //4
    AutoTrajectory R9Backup2 =
            OneL4CoralMidRoutine.trajectory("R9Backup2");


    OneL4CoralMidRoutine.active().onTrue(
            sequence(
                    Commands.print("Started" +
                            "OneL4CoralMidRoutine" +
                            " the routine!"),
                    StarttoR9.resetOdometry(),
                    StarttoR9.cmd()
            )
    );

    StarttoR9.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), R9Forward.cmd()));
    //Go against R9, then backup a lil bit
    R9Forward.done().onTrue(sequence(stop(), score(), parallel(R9Backup2.cmd(), lowerArmElevatorToOrigin())));

    return OneL4CoralMidRoutine;
    }

    public AutoRoutine OneL4CoralMidRoutineTopStart() {

        //        var trajectory = loadTrajectory(
        //                "FourL4CoralBottom");

        AutoRoutine OneL4CoralMidRoutineTopStart =
                autoFactory.newRoutine(
                        "OneL4CoralMidRoutineTopStart");
        //Initialize
        //1
        AutoTrajectory StarttoR9Top =
                OneL4CoralMidRoutineTopStart.trajectory("StarttoR9Top");
        //2
        AutoTrajectory R9Backup =
                OneL4CoralMidRoutineTopStart.trajectory("R9Backup");
        //3
        AutoTrajectory R9Forward =
                OneL4CoralMidRoutineTopStart.trajectory("R9Forward");
        //4
        AutoTrajectory R9BackupSecond =
                OneL4CoralMidRoutineTopStart.trajectory("R9Backup2");


        OneL4CoralMidRoutineTopStart.active().onTrue(
                sequence(
                        Commands.print("Started" +
                                "OneL4CoralMidRoutineTopStart" +
                                " the routine!"),
                        StarttoR9Top.resetOdometry(),
                        StarttoR9Top.cmd()
                )
        );

        StarttoR9Top.done().onTrue(sequence(stop(), (R9Backup.cmd())));
        //Go against R9, then backup a lil bit
        R9Backup.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), (R9Forward.cmd())));
        R9Forward.done().onTrue(sequence(stop(), score(), R9BackupSecond.cmd()));
        R9BackupSecond.active().onTrue(lowerArmElevatorToOrigin());
        return OneL4CoralMidRoutineTopStart;
    }
//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------
public AutoRoutine OneL4CoralMidRoutineBottomStart() {

    //        var trajectory = loadTrajectory(
    //                "FourL4CoralBottom");

    AutoRoutine OneL4CoralMidRoutineBottomStart =
            autoFactory.newRoutine(
                    "OneL4CoralMidRoutineBottomStart");
    //Initialize
    //1
    AutoTrajectory StarttoR9Bottom =
            OneL4CoralMidRoutineBottomStart.trajectory("StarttoR9Bottom");
    //2
    AutoTrajectory R9Backup =
            OneL4CoralMidRoutineBottomStart.trajectory("R9Backup");
    //3
    AutoTrajectory R9Forward =
            OneL4CoralMidRoutineBottomStart.trajectory("R9Forward");
    //4



    OneL4CoralMidRoutineBottomStart.active().onTrue(
            sequence(
                    Commands.print("Started" +
                            "OneL4CoralMidRoutine" +
                            " the routine!"),
                    StarttoR9Bottom.resetOdometry(),
                    StarttoR9Bottom.cmd()
            )
    );

    StarttoR9Bottom.done().onTrue(sequence(stop(), (R9Backup.cmd())));
    //Go against R9, then backup a lil bit
    R9Backup.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), (R9Forward.cmd())));
    //Pull forward then score
    R9Forward.done().onTrue(sequence(stop(), score() , lowerArmElevatorToOrigin()));


    return OneL4CoralMidRoutineBottomStart;
}

    public AutoRoutine OneL4CoralTopRoutine() {

        //        var trajectory = loadTrajectory(
        //                "FourL4CoralBottom");

        AutoRoutine OneL4CoralTopRoutine =
                autoFactory.newRoutine(
                        "OneL4CoralTopRoutine");
        //Initialize
        //1
        AutoTrajectory StarttoR12 =
                OneL4CoralTopRoutine.trajectory("StarttoR12");
        //2
        AutoTrajectory R12Backup =
                OneL4CoralTopRoutine.trajectory("R12Backup");
        //3
        AutoTrajectory R12Forward =
                OneL4CoralTopRoutine.trajectory("R12Forward");
        //4



        OneL4CoralTopRoutine.active().onTrue(
                sequence(
                        Commands.print("Started" +
                                "OneL4CoralTopRoutine" +
                                " the routine!"),
                        StarttoR12.resetOdometry(),
                        StarttoR12.cmd()
                )
        );

        StarttoR12.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), (R12Forward.cmd())));
        R12Forward.done().onTrue(sequence(stop(), score() , lowerArmElevatorToOrigin()));

        return OneL4CoralTopRoutine;
    }

    public AutoRoutine OneL4CoralBottomRoutine() {

        //        var trajectory = loadTrajectory(
        //                "FourL4CoralBottom");

        AutoRoutine OneL4CoralBottomRoutine =
                autoFactory.newRoutine(
                        "OneL4CoralBottomRoutine");
        //Initialize
        //1
        AutoTrajectory StarttoR7 =
                OneL4CoralBottomRoutine.trajectory("StarttoR7");
        //2
        AutoTrajectory R7Backup =
                OneL4CoralBottomRoutine.trajectory("R7Backup");
        //3
        AutoTrajectory R7Forward =
                OneL4CoralBottomRoutine.trajectory("R7Forward");
        //4



        OneL4CoralBottomRoutine.active().onTrue(
                sequence(
                        Commands.print("Started" +
                                "OneL4CoralBottomRoutine" +
                                " the routine!"),
                        StarttoR7.resetOdometry(),
                        StarttoR7.cmd()
                )
        );

        StarttoR7.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), R7Forward.cmd()));
        R7Forward.done().onTrue(sequence(stop(), score() , lowerArmElevatorToOrigin()));

        return OneL4CoralBottomRoutine;
    }
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    public Command OneL3Score() {
        return sequence(
                Commands.deadline(waitSeconds(4), new RobotRelativeDriveCommand(drive, () -> 0.2, () -> 0, () -> 0)),
                waitSeconds(1),
                Commands.deadline(waitSeconds(0.2), new RobotRelativeDriveCommand(drive, () -> -0.2, () -> 0, () -> 0)),
                Commands.deadline(waitSeconds(3), new ArmElevatorToSetpoint(elevator, arm, ElevatorConstants.L3 + 0.005, ArmConstants.armL4Angle + Units.degreesToRadians(1))),
                Commands.deadline(waitSeconds(0.05), new RobotRelativeDriveCommand(drive, () -> 0.1, () -> 0, () -> 0)),
                parallel(
                        new ArmElevatorToOrigin(elevator, arm),
                        Commands.deadline(waitSeconds(0.2), new RobotRelativeDriveCommand(drive, () -> -0.1, () -> 0, () -> 0))
                        )


                );
    }

    public Command TaxiBack() {
        return sequence(
                waitSeconds(0),
                Commands.deadline(
                        waitSeconds(3),
                        new RobotRelativeDriveCommand(drive, () -> -1, () -> 0, () -> 0)
                )
        );
    };

    public Command TaxiForward() {
        return sequence(
                waitSeconds(0),
                Commands.deadline(
                        waitSeconds(3),
                        new RobotRelativeDriveCommand(drive, () -> 1, () -> 0, () -> 0)
                )
        );
    };

    public AutoRoutine TestingFR() {
        AutoRoutine testing = autoFactory.newRoutine("Testing");
        AutoTrajectory circle = testing.trajectory("Circle");

        testing.active().onTrue(circle.resetOdometry().andThen(circle.cmd()));
        circle.done().onTrue(Commands.run(drive::stop, drive));
        return testing;
    }
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
