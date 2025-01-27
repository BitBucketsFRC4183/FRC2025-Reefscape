package frc.robot.subsystems.Auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoSim extends AutoUtil{
    private static AutoFactory AutoFactorySim;
    private static AutoChooser AutoChooserSim;

    public static AutoChooser getChooser() {

        return AutoChooserSim;
    }

    public static AutoFactory getAutoFactory() {

        return AutoFactorySim;
    }

    private static void setupFactory() {
        AutoFactorySim.bind("Marker", Commands.print(
                "Marker Passed"));
    }

    private static void setupChooser() {

        AutoChooser AutoChooserSim =
                new AutoChooser();
        AutoChooserSim.addRoutine("FourL4CoralBottomRoutine", AutoSubsystem::FourL4CoralBottomRoutine);
        AutoChooserSim.addRoutine("FourL4CoralTopRoutine", AutoSubsystem::FourL4CoralBottomRoutine);
        AutoChooserSim.addRoutine("OneL4CoralMid", AutoSubsystem::FourL4CoralBottomRoutine);
        AutoChooserSim.addCmd("drive",
                AutoSubsystem::drive);
        AutoChooserSim.addCmd("deposit",
                AutoSubsystem::deposit);
        AutoChooserSim.addCmd("claw",
                AutoSubsystem::claw);
        SmartDashboard.putData(AutoChooserSim);


    }


}
