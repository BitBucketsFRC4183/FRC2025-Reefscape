package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorInput {
  final CommandXboxController operator = new CommandXboxController(1);
  public final CommandXboxController driver = new CommandXboxController(0);


  // DRIVER'S CONTROLS
  public final Trigger slowModeHold = driver.leftTrigger();
  public final Trigger turboModeHold = driver.rightTrigger();


  final Trigger elevatorGoToOrigin = operator.a();

  final Trigger elevatorsetpoint1 = operator.x();
  final Trigger elevatorsetpoint2 = operator.y();
  final Trigger elevatorsetpoint3 = operator.b();
  final Trigger manualElevator = operator.axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, 0.1);
  final Trigger resetElevatorEncoder = operator.start();

  //final Trigger manualArmCommand = operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.1);
  final Trigger manualPivotCommand = operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.1);
  final Trigger openClaw = driver.leftStick();
  final Trigger closeClaw = driver.rightStick();

  final Trigger IntakeOn = operator.a();

  // final Trigger armbendup = operator.povUp();
  // final Trigger armbenddown = operator.povDown();

  final Trigger xNotDesired =
      driver
          .axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1)
          .or(driver.axisLessThan(XboxController.Axis.kLeftX.value, -0.1))
          .negate();
  final Trigger yNotDesired =
      driver
          .axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
          .or(driver.axisLessThan(XboxController.Axis.kLeftY.value, -0.1))
          .negate();
  final Trigger thetaNotDesired =
      driver
          .axisGreaterThan(XboxController.Axis.kRightX.value, 0.1)
          .or(driver.axisLessThan(XboxController.Axis.kRightX.value, -0.1))
          .negate();

  public final Trigger movementNotDesired = xNotDesired.and(yNotDesired).and(thetaNotDesired);
  public final Trigger movementDesired = movementNotDesired.negate();

  /**
   * @param input a value
   * @return that value, deadbanded
   */
  static double deadband(double input) {
    double value = input;

    value = MathUtil.applyDeadband(value, 0.1);
    value = Math.copySign(value * value, value);

    return value;
  }


}
