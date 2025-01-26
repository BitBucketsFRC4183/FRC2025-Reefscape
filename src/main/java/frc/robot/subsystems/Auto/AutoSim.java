package frc.robot.subsystems.Auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoSim extends AutoUtil{
    private static AutoFactory factorySim;
    private static AutoChooser chooserSim;

    public static AutoChooser getChooser() {

        return chooserSim;
    }

    public static AutoFactory getAutoFactory() {

        return factorySim;
    }

    private static void setupFactory() {
        factorySim.bind("Marker", Commands.print(
                "Marker Passed"));
    }

    private static void setupChooser() {

        AutoChooser autoChooserSim =
                new AutoChooser();
        autoChooserSim.addRoutine("FourL4CoralBottomRoutine", AutoSubsystem::FourL4CoralBottomRoutine);
        autoChooserSim.addCmd("drive",
                AutoSubsystem::drive);
        autoChooserSim.addCmd("deposit",
                AutoSubsystem::deposit);
        autoChooserSim.addCmd("claw",
                AutoSubsystem::claw);
        SmartDashboard.putData(autoChooserSim);


    }


}
