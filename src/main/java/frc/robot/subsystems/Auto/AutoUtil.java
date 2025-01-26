package frc.robot.subsystems.Auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;


public class AutoUtil {
    private static AutoFactory factory;
    private static AutoChooser chooser;

    public static void initAuto() {
        setupFactory();
        setupChooser();
    }

    public static AutoChooser getChooser() {

        return chooser;
    }

    public static AutoFactory getAutoFactory() {

        return factory;
    }

    private static void setupFactory() {
        factory.bind("Marker", Commands.print(
                "Marker Passed"));
    }

    private static void setupChooser() {
        // interface for choreo

        // Made sendable, use SmartDashbaord now
//        chooser.addCmd("My Routine", () -> Autos.getMyRoutine());
//        chooser.addCmd("Print", () -> Autos.getPrint());
//        chooser.addCmd("Split", () -> Autos.getSplitRoutine());
//        chooser.addCmd("Straight", () -> Autos.getStraight());
//        // Default
//        chooser.select("Straight");

        AutoChooser autoChooser = new AutoChooser();
        autoChooser.addRoutine("FourL4CoralBottomRoutine", AutoSubsystem::FourL4CoralBottomRoutine);
        autoChooser.addCmd("drive",
                AutoSubsystem::drive);
        autoChooser.addCmd("deposit",
                AutoSubsystem::deposit);
        autoChooser.addCmd("claw",
                AutoSubsystem::claw);
        SmartDashboard.putData(autoChooser);


    }

    }

