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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmCommands.ArmHoverCommand;
import frc.robot.commands.ArmCommands.ManualArmCommand;
import frc.robot.commands.BaseDriveCommand;
import frc.robot.commands.ArmCommands.BendCommand;
import frc.robot.commands.ElevatorCommands.ElevatorGoToOriginCommand;
import frc.robot.commands.ElevatorCommands.ElevatorSetPointCommand;
import frc.robot.commands.ElevatorCommands.ManualElevatorCommand;
import frc.robot.commands.ElevatorCommands.ResetElevatorEncoderCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeManagementSubsystem.AlgaeManagementSubsystem;
import frc.robot.subsystems.Auto.AutoSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem.*;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.GyroIO;
import frc.robot.subsystems.DriveSubsystem.GyroIOPigeon2;
import frc.robot.subsystems.DriveSubsystem.ModuleIO;
import frc.robot.subsystems.DriveSubsystem.ModuleIOSim;
import frc.robot.subsystems.ElevatorSubsystem.*;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundIntakeSubsystem;
import frc.robot.subsystems.LEDSubsytem.LEDSubsystem;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmIOSim;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmSparkMax;
import frc.robot.subsystems.SingleJointedArmSubsystem.SingleJointedArmSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionIO;
import frc.robot.subsystems.VisionSubsystem.VisionIOPhotonVision;
import frc.robot.subsystems.VisionSubsystem.VisionIOPhotonVisionSim;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.DoubleSupplier;

public class RobotContainer {
  // Subsystems
  public final DriveSubsystem drive;
  // private final Flywheel flywheel;
  public final OperatorInput operatorInput;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeManagementSubsystem algaeManagementSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final GroundIntakeSubsystem groundIntakeSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final SingleJointedArmSubsystem singleJointedArmSubsystem;
  private final VisionSubsystem visionSubsystem;
  private SwerveDriveSimulation driveSimulation = null;
  private final AutoSubsystem autoSubsystem;



  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController elevatorAndArmController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.operatorInput = new OperatorInput();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new DriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOHybrid(0, TunerConstants.FrontLeft),
                new ModuleIOHybrid(1,TunerConstants.FrontRight),
                new ModuleIOHybrid(2, TunerConstants.BackLeft),
                new ModuleIOHybrid(3, TunerConstants.BackRight));

        elevatorSubsystem =
                new ElevatorSubsystem(new ElevatorIOSparkMax(), new ElevatorEncoderIOSim()); //TODO
        algaeManagementSubsystem =
                new AlgaeManagementSubsystem(); //TODO
        clawSubsystem =
                new ClawSubsystem(); //TODO
        groundIntakeSubsystem =
                new GroundIntakeSubsystem(); //TODO
        ledSubsystem =
                new LEDSubsystem(); //TODO
        singleJointedArmSubsystem =
                new SingleJointedArmSubsystem(new SingleJointedArmSparkMax()) {
                }; //TODO
        visionSubsystem =
                new VisionSubsystem(new VisionIOPhotonVision()); //TODO
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        this.driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d());
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new DriveSubsystem(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOSim(driveSimulation.getModules()[0]),
                    new ModuleIOSim(driveSimulation.getModules()[1]),
                    new ModuleIOSim(driveSimulation.getModules()[2]),
                    new ModuleIOSim(driveSimulation.getModules()[3]));
//        new DriveSubsystem(
//                new GyroIOSim(driveSimulation.getGyroSimulation()),
//                new ModuleIOHybridSim(0, TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
//                new ModuleIOHybridSim(1, TunerConstants.FrontRight,driveSimulation.getModules()[1]),
//                new ModuleIOHybridSim(2, TunerConstants.BackLeft,driveSimulation.getModules()[2]),
//                new ModuleIOHybridSim(3, TunerConstants.BackRight,driveSimulation.getModules()[3]));
        // flywheel = new Flywheel(new FlywheelIOSim());
        elevatorSubsystem =
                new ElevatorSubsystem(new ElevatorIOSim(), new ElevatorEncoderIOSim()); //TODO
        algaeManagementSubsystem =
                new AlgaeManagementSubsystem(); //TODO
        clawSubsystem =
                new ClawSubsystem(); //TODO
        groundIntakeSubsystem =
                new GroundIntakeSubsystem(); //TODO
        ledSubsystem =
                new LEDSubsystem(); //TODO
        singleJointedArmSubsystem =
                new SingleJointedArmSubsystem(new SingleJointedArmIOSim()); //TODO
        visionSubsystem =
                new VisionSubsystem(new VisionIOPhotonVisionSim(driveSimulation::getSimulatedDriveTrainPose)); //TODO
        break;
      default:
        // Replayed robot, disable IO implementations
        drive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // flywheel = new Flywheel(new FlywheelIO() {});
        elevatorSubsystem =
                new ElevatorSubsystem(new ElevatorIO() {}, new ElevatorEncoderIO() {}); //TODO
        algaeManagementSubsystem =
                new AlgaeManagementSubsystem(); //TODO
        clawSubsystem =
                new ClawSubsystem(); //TODO
        groundIntakeSubsystem =
                new GroundIntakeSubsystem(); //TODO
        ledSubsystem =
                new LEDSubsystem(); //TODO
        singleJointedArmSubsystem =
                new SingleJointedArmSubsystem(new SingleJointedArmIOSim()); //TODO
        visionSubsystem =
                new VisionSubsystem(new VisionIO() {}); //TODO
        break;
    }

    // Set up auto routines

    autoSubsystem = new AutoSubsystem(drive);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "DriveSubsystem SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "DriveSubsystem SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "DriveSubsystem SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "DriveSubsystem SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    loadCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  void loadCommands() {

    singleJointedArmSubsystem.setDefaultCommand(new ArmHoverCommand(singleJointedArmSubsystem));
    operatorInput.elevatorsetpoint1.whileTrue(new ElevatorSetPointCommand(elevatorSubsystem, ElevatorConstants.L1));
    operatorInput.elevatorsetpoint2.whileTrue(new ElevatorSetPointCommand(elevatorSubsystem, ElevatorConstants.L3));
    operatorInput.elevatorsetpoint3.whileTrue(new ElevatorSetPointCommand(elevatorSubsystem, ElevatorConstants.L4));

    operatorInput.elevatorGoToOrigin.onTrue(new ElevatorGoToOriginCommand(elevatorSubsystem));

    operatorInput.armbendup.whileTrue(new BendCommand(singleJointedArmSubsystem, Math.PI/2));
    operatorInput.armbenddown.whileTrue(new BendCommand(singleJointedArmSubsystem, -Math.PI/2));

    operatorInput.manualArmCommand.whileTrue(new ManualArmCommand(singleJointedArmSubsystem, elevatorAndArmController::getRightY));

    operatorInput.resetElevatorEncoder.onTrue(new ResetElevatorEncoderCommand(elevatorSubsystem));

    operatorInput.manualElevator.whileTrue(new ManualElevatorCommand(elevatorSubsystem, elevatorAndArmController::getLeftY));


    operatorInput.movementDesired.whileTrue(
            new BaseDriveCommand.basedrivecommand(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX()));
  } // TODO FIX COMMAND THIS WILL BREAK DO NOT RUN IT

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
}
