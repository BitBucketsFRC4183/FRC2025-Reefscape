package frc.robot.subsystems.AutoSubsystem;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ArmCommands.ArmToSetpoint;
import frc.robot.commands.ArmElevatorToOrigin;
import frc.robot.commands.ArmElevatorToSetpoint;
import frc.robot.commands.DriveCommands.RobotRelativeDriveCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constants.ArmConstants.MIN_ANGLE_RADS;


public class AutoSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final AutoFactory autoFactory;

    private static Choreo.TrajectoryLogger<SwerveSample> trajectoryLogger() {
        return (swerveSampleTrajectory, aBoolean) -> {
            Logger.recordOutput(
                      "Odometry/Trajectory", swerveSampleTrajectory.getPoses()
            );
            Logger.recordOutput("Odometry/TrajectoryGoal", swerveSampleTrajectory.getFinalPose(false).get());
        };
    }
    public AutoSubsystem(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.drive = drive;
        this.elevator = elevator;
        this.arm = arm;
        this.autoFactory = new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectorySample, false, drive, trajectoryLogger());

    }

    public Command raiseArmElevatorToL4() {
        return deadline(waitSeconds(1.25), new ArmElevatorToSetpoint(elevator, arm, ElevatorConstants.L4, ArmConstants.armL4Angle));
    }
    public Command lowerArmElevatorToOrigin() {
        return deadline(waitSeconds(1.5), new ArmElevatorToOrigin(elevator,arm));
    }
    public Command stop() {
        return Commands.runOnce(drive::stop);
    }
    public Command score() {
        return Commands.deadline(waitSeconds(0.8), new ArmElevatorToSetpoint(elevator, arm, ElevatorConstants.L3, Units.degreesToRadians(-20)));
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
                sequence(
                        Commands.print("Started" +
                                "ThreeL4CoralTopRoutine" +
                                " the routine!"),
                        StarttoR11.resetOdometry(),
                        StarttoR11.cmd()

                )
        );

        StarttoR11.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), score(), parallel(R11toSource.cmd(), lowerArmElevatorToOrigin())));

        R11toSource.done().onTrue(sequence(stop(), waitSeconds(0.5), SourcetoR12.cmd()));

        SourcetoR12.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), score() , parallel(R12toSource.cmd(), lowerArmElevatorToOrigin())));

        R12toSource.done().onTrue(sequence(stop(), waitSeconds(0.5), SourcetoR1.cmd()));

        SourcetoR1.done().onTrue(sequence(stop(), raiseArmElevatorToL4(), (score())));

        

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
    AutoTrajectory R9toStart =
            OneL4CoralMidRoutine.trajectory("R9toStart");


    OneL4CoralMidRoutine.active().onTrue(
            sequence(
                    Commands.print("Started" +
                            "OneL4CoralMidRoutine" +
                            " the routine!"),
                    StarttoR9.resetOdometry(),
                    StarttoR9.cmd().alongWith(raiseArmElevatorToL4())
            )
    );

    StarttoR9.done().onTrue(Commands.run(drive::stop, drive).andThen(raiseArmElevatorToL4().andThen(score().andThen(lowerArmElevatorToOrigin()))));
    //StarttoR9.done().onTrue(R9toStart.cmd().alongWith(lowerArmElevatorToOrigin()));
    R9toStart.done().onTrue(Commands.run(drive::stop, drive));

    return OneL4CoralMidRoutine;
    }


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
