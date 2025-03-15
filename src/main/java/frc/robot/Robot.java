// Copyright 2021-2025 FRC 6328
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
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Constants;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonVersion;

import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private AutoChooser autoChooser;
  private Trajectory m_trajectory;
  private Field2d m_field;

  public Robot() {

    //SmartDashboard.putData(autoChooser);
    //RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    // Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit logger
    Logger.start();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  private final Timer timer = new Timer();

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    
    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    m_trajectory =
            TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                    new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                    new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    m_field.getObject("traj").setTrajectory(m_trajectory);
  }

  /**
   * This function is called periodically during all modes.
   */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() meth
    // ods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    robotContainer.resetSimulationField();
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }


  @Override
  public void disabledExit() {
  }


  @Override
  public void autonomousInit() {

//        System.out.println(Arrays.toString(Choreo.availableTrajectories()));
//        var trajectory = loadTrajectory(
//                "FourL4CoralBottom");
//
//        if (trajectory.isPresent()) {
//          System.out.print("THANK");
//          // Get the initial pose of the trajectory
//          Optional<Pose2d> initialPose =
//          trajectory.get().getInitialPose
//          (isRedAlliance());
//          Logger.recordOutput("monkey", true);
//          if (initialPose.isPresent()) {
//            // Reset odometry to the start of the trajectory
//            robotContainer.drive.setPose(initialPose.get());
//          }
//        }
////     Reset and start the timer when the autonomous period begins
//        timer.restart();
//    robotContainer.getAutonomousCommand().execute();
  }

  @Override
  public void autonomousPeriodic() {
//    Optional<Trajectory<SwerveSample>> trajectory = loadTrajectory("FourL4CoralBottom");
//    if (trajectory.isPresent()) {
//      Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());
//
//      sample.ifPresent(swerveSample -> robotContainer.drive.followTrajectorySample(swerveSample));
//    }
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
  }

  @Override
  public void autonomousExit(){

  }




  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit(){

  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit(){
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
    robotContainer.resetSimulationField();
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    robotContainer.displaySimFieldToAdvantageScope();
  }



}

