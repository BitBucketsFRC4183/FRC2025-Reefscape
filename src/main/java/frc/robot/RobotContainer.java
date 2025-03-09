// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.math.geometry.Pose3d;

//import frc.robot.commands.DriveCommands;
//import frc.robot.commands.ResetEncoderCommand;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmCommands.ArmBendCommand;
import frc.robot.commands.ArmCommands.ArmHoverCommand;
import frc.robot.commands.ArmCommands.ManualArmCommand;
import frc.robot.commands.ArmElevatorToOrigin;
import frc.robot.commands.ArmElevatorToSetpoint;
import frc.robot.commands.DriveCommands.FieldDriveElevatorLimitedCommand;
import frc.robot.commands.DriveCommands.RobotRelativeDriveCommand;
import frc.robot.commands.ElevatorCommands.ElevatorSetPointCommand;
import frc.robot.commands.ElevatorCommands.ManualElevatorCommand;
import frc.robot.commands.ElevatorCommands.ResetElevatorEncoderCommand;
import frc.robot.commands.DriveCommands.FieldRelativeDriveCommand;
import frc.robot.commands.IntakeCommands.IntakeSetRollersCommand;
import frc.robot.commands.DriveCommands.ResetHeadingCommand;
import frc.robot.commands.IntakeCommands.PivotDownCommand;
import frc.robot.commands.IntakeCommands.PivotUpCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeIO;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.AutoSubsystem.AutoSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.ClawSubsystem.*;
import frc.robot.subsystems.DriveSubsystem.*;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.GyroIO;
import frc.robot.subsystems.DriveSubsystem.GyroIOPigeon2;
import frc.robot.subsystems.DriveSubsystem.ModuleIO;
import frc.robot.subsystems.DriveSubsystem.ModuleIOSim;
import frc.robot.subsystems.ElevatorSubsystem.*;
import frc.robot.subsystems.IntakeSubsystem.IntakeIOSim;
import frc.robot.subsystems.IntakeSubsystem.IntakeIOSparkMax;
import frc.robot.subsystems.LEDSubsytem.LEDSubsystem;
import frc.robot.subsystems.ArmSubsystem.*;
import frc.robot.subsystems.VisionSubsystem.VisionIO;
import frc.robot.subsystems.VisionSubsystem.VisionIOPhotonVisionSim;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoral;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;


public class RobotContainer {
  // Subsystems
  public final DriveSubsystem driveSubsystem;
  public final OperatorInput operatorInput;
  private final ElevatorSubsystem elevatorSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final ArmSubsystem armSubsystem;
  private final VisionSubsystem visionSubsystem;
  public static SwerveDriveSimulation driveSimulation = null;
  private final AutoSubsystem autoSubsystem;
  private AutoChooser autoChooser;

  // Controllers
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final SlewRateLimiter slewX = new SlewRateLimiter(DriveConstants.slewX);
  private final SlewRateLimiter slewY = new SlewRateLimiter(DriveConstants.slewY);
  private final SlewRateLimiter slewTheta = new SlewRateLimiter(DriveConstants.slewTheta);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    this.operatorInput = new OperatorInput();



    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        driveSubsystem =
                new DriveSubsystem(
                        new GyroIOPigeon2(),
                        new ModuleIOHybrid(0, TunerConstants.FrontLeft),
                        new ModuleIOHybrid(1, TunerConstants.FrontRight),
                        new ModuleIOHybrid(2, TunerConstants.BackLeft),
                        new ModuleIOHybrid(3, TunerConstants.BackRight));
        elevatorSubsystem =
                new ElevatorSubsystem(new ElevatorIOSparkMax(),
                        new ElevatorEncoderIOThroughbore());
        clawSubsystem =
                new ClawSubsystem(new EndEffectorIO() {}); // NO HARDWARE LOL
        intakeSubsystem =
                new IntakeSubsystem(new IntakeIOSparkMax());
        ledSubsystem =
                new LEDSubsystem();
        armSubsystem =
                new ArmSubsystem(new ArmIOTalonFX(), new ArmEncoderIOThroughbore());
        visionSubsystem =
                new VisionSubsystem(new VisionIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d());
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        driveSubsystem =
            new DriveSubsystem(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOSim(driveSimulation.getModules()[0]),
                    new ModuleIOSim(driveSimulation.getModules()[1]),
                    new ModuleIOSim(driveSimulation.getModules()[2]),
                    new ModuleIOSim(driveSimulation.getModules()[3]));

        ElevatorIOSim elevatorIOSim = new ElevatorIOSim();
        elevatorSubsystem =
                new ElevatorSubsystem(elevatorIOSim, new ElevatorEncoderIOSim(elevatorIOSim.elevatorMotor1Sim));
        clawSubsystem =
                new ClawSubsystem(new EndEffectorIOSim());
        intakeSubsystem =
                new IntakeSubsystem(new IntakeIOSim(driveSimulation));
        ledSubsystem =
                new LEDSubsystem();
        armSubsystem =
                new ArmSubsystem(new ArmIOSim(), new ArmEncoderIO() {});
        visionSubsystem =
                new VisionSubsystem(new VisionIOPhotonVisionSim(driveSimulation::getSimulatedDriveTrainPose));
        break;


      default:
        // Replayed robot, disable IO implementations
        driveSubsystem =
                new DriveSubsystem(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
        elevatorSubsystem =
                new ElevatorSubsystem(new ElevatorIO() {}, new ElevatorEncoderIO() {}); //TODO
        clawSubsystem =
                new ClawSubsystem(new EndEffectorIO() {});
        intakeSubsystem =
                new IntakeSubsystem(new IntakeIO() {});
        ledSubsystem =
                new LEDSubsystem(); //TODO
        armSubsystem =
                new ArmSubsystem(new ArmIO() {}, new ArmEncoderIO() {}); //TODO
        visionSubsystem =
                new VisionSubsystem(new VisionIO() {
                }); //TODO

        break;
    }

    this.autoSubsystem = new AutoSubsystem(driveSubsystem, elevatorSubsystem, armSubsystem);
    autoChooser = new AutoChooser();

//    autoChooser.addRoutine("FourL4CoralBottom", AutoSubsystem::FourL4CoralBottomRoutine);
//    autoChooser.addRoutine("FourL4CoralTop", AutoSubsystem::FourL4CoralTopRoutine);
//    autoChooser.addRoutine("ThreeL4CoralBottom", AutoSubsystem::ThreeL4CoralBottomRoutine);
//    autoChooser.addRoutine("ThreeL4CoralTop", AutoSubsystem::ThreeL4CoralTopRoutine);
//    autoChooser.addRoutine("OneL4CoralMid", AutoSubsystem::OneL4CoralMidRoutine);
    autoChooser.addCmd("Score1L4Coral", autoSubsystem::OneL4Score);
    autoChooser.addCmd("nothing", Commands::none);
    autoChooser.addCmd("TaxiBack", autoSubsystem::TaxiBack);
//    autoChooser.addCmd("DriveSysIDQuasistaticForward", () -> driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//    autoChooser.addCmd("DriveSysIDQuasistaticReverse", () -> driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//    autoChooser.addCmd("DriveSysIDDynamicForward", () -> driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
//    autoChooser.addCmd("DriveSysIDDynamicReverse", () -> driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
//    autoChooser.addCmd("WheelBaseCharacterization", () -> new WheelBaseCharacterizationRoutineCommand(driveSubsystem));

//    autoChooser.addCmd("ElevatorSysIDQuasistaticForward", () -> elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//    autoChooser.addCmd("ElevatorSysIDQuasistaticReverse", () -> elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//    autoChooser.addCmd("ElevatorSysIDDynamicForward", () -> elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
//    autoChooser.addCmd("ElevatorSysIDDynamicReverse", () -> elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
//
//    autoChooser.addCmd("ArmSysIDQuasistaticForward", () -> armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//    autoChooser.addCmd("ArmSysIDQuasistaticReverse", () -> armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//    autoChooser.addCmd("ArmSysIDDynamicForward", () -> armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
//    autoChooser.addCmd("ArmSysIDDynamicReverse", () -> armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("autochooser", autoChooser);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    loadCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * Joystick} or {@link XboxController}), and then passing it to a {@link
   * JoystickButton}.
   */
  void loadCommands() {


    // elevator stuff
    operatorInput.armElevatorOrigin.whileTrue(new ArmElevatorToOrigin(elevatorSubsystem, armSubsystem));
    operatorInput.armElevatorL2.whileTrue(new ArmElevatorToSetpoint(elevatorSubsystem, armSubsystem, ElevatorConstants.L2, ArmConstants.armL2Angle));
    operatorInput.armElevatorL3.whileTrue(new ArmElevatorToSetpoint(elevatorSubsystem, armSubsystem, ElevatorConstants.L3, ArmConstants.armL3Angle));
    operatorInput.armElevatorL4.whileTrue(new ArmElevatorToSetpoint(elevatorSubsystem, armSubsystem, ElevatorConstants.L4, ArmConstants.armL4Angle));
    operatorInput.resetElevatorEncoder.onTrue(new ResetElevatorEncoderCommand(elevatorSubsystem));
    // operatorInput.armElevatorL4.whileTrue(new ElevatorSetPointCommand(elevatorSubsystem, ElevatorConstants.L3));
    //arm stuff
    armSubsystem.setDefaultCommand(new ArmHoverCommand(armSubsystem));
    // operatorInput.armSetpointUp.whileTrue(new ArmBendCommand(armSubsystem, 0));
    // operatorInput.armSetpointDown.whileTrue(new ArmBendCommand(armSubsystem, ArmConstants.setpointDown));


    // manual commands

    // uncomment and comment which ever one is needed
    operatorInput.manualArm.whileTrue(new ManualArmCommand(armSubsystem, operatorController::getRightY));
    // operatorInput.manualPivot.whileTrue(new IntakeSetPivotCommand(intakeSubsystem, operatorController::getRightY));
    operatorInput.manualElevator.whileTrue(new ManualElevatorCommand(elevatorSubsystem, operatorController::getLeftY));

    // rollers and pivot, algae intake stuff
    operatorInput.rollersIn.whileTrue(new IntakeSetRollersCommand(intakeSubsystem, false));
    operatorInput.rollersOut.whileTrue(new IntakeSetRollersCommand(intakeSubsystem, true));
    operatorInput.rollerPivotDown.whileTrue(new PivotDownCommand(intakeSubsystem));
    operatorInput.rollerPivotUp.whileTrue(new PivotUpCommand(intakeSubsystem));

    // claw stuff
    // operatorInput.openClaw.onTrue(new OpenClawCommand(clawSubsystem));
    // operatorInput.closeClaw.onTrue(new CloseClawCommand(clawSubsystem));


    // drive stuff
    operatorInput.resetHeading.onTrue(new ResetHeadingCommand(driveSubsystem));
    operatorInput.movementDesired.whileTrue(
            new FieldDriveElevatorLimitedCommand(
                    driveSubsystem,
                () -> slewX.calculate(driveController.getLeftY()),
                () -> slewY.calculate(driveController.getLeftX()),
                () -> slewTheta.calculate(-driveController.getRightX()),
                    driveSubsystem::getRotation,
                    elevatorSubsystem
    ));

    DoubleSupplier dPadY = () -> {
      if (driveController.povLeft().getAsBoolean()) {
        return 1.0;
      } else if (driveController.povRight().getAsBoolean()) {
        return -1.0;
      } else { return 0.0;}
    };

    DoubleSupplier dPadX = () -> {
      if (driveController.povDown().getAsBoolean()) {
        return -1.0;
      } else if (driveController.povUp().getAsBoolean()) {
        return 1.0;
      } else { return 0.0;}
    };

    OperatorInput.alignmentRobotRelative.whileTrue(
            new RobotRelativeDriveCommand(
                    driveSubsystem,dPadX, dPadY,() -> 0
            )
    );
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    driveSubsystem.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void addCoral() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoral(new Pose2d(2, 2, Rotation2d.fromDegrees(90))));
    Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }

  public void periodic() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Pose3d[] coralPoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Coral");
    Logger.recordOutput("FieldSimulation/CoralPositions", coralPoses);
  }
}
