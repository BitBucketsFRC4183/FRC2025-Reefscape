package frc.robot.subsystems.AutoSubsystem;

import choreo.auto.AutoChooser;


public class AutoUtil {
    AutoChooser autoChooser = new AutoChooser();
    /*private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;
    public initAuto() {
        AutoChooser autoChooser = new AutoChooser();
        AutoChooser.addRoutine("FourL4CoralBottomRoutine", AutoSubsystem::FourL4CoralBottomRoutine);
        AutoChooser.addRoutine("FourL4CoralTopRoutine", AutoSubsystem::FourL4CoralBottomRoutine);
        AutoChooser.addRoutine("OneL4CoralMidRoutine", AutoSubsystem::FourL4CoralBottomRoutine);
        AutoChooser.addCmd("drive",
                AutoSubsystem::drive);
        AutoChooser.addCmd("deposit",
                AutoSubsystem::deposit);
        AutoChooser.addCmd("claw",
                AutoSubsystem::claw);
        SmartDashboard.putData(AutoChooser);*/


        /*setupFactory();
        setupChooser();*/
    //}
    

    private static void setupFactory() {
        /*AutoFactory.bind("Marker", Commands.print(
                "Marker Passed"));*/
    }

    private static void setupChooser() {}
        // interface for choreo

        // Made sendable, use SmartDashboard now
//        chooser.addCmd("My Routine", () -> Autos.getMyRoutine());
//        chooser.addCmd("Print", () -> Autos.getPrint());
//        chooser.addCmd("Split", () -> Autos.getSplitRoutine());
//        chooser.addCmd("Straight", () -> Autos.getStraight());
//        // Default
//        chooser.select("Straight");


    }

