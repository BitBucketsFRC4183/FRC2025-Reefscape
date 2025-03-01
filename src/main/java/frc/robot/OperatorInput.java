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


  //operator controls
  final Trigger elevatorGoToOrigin = operator.a();

  final Trigger elevatorSetpoint1 = operator.x();
  final Trigger elevatorSetpoint2 = operator.y();
  final Trigger elevatorSetpoint3 = operator.b();
  final Trigger manualElevator = operator.axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, 0.1);
  final Trigger resetElevatorEncoder = operator.start();

  final Trigger manualArm = operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.1);
  final Trigger manualPivot = operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.1);
  // final Trigger openClaw = driver.leftStick();
  // final Trigger closeClaw = driver.rightStick();

  final Trigger rollersIn = operator.povRight();
  final Trigger rollersOut = operator.povLeft();

  final Trigger armSetpointUp = operator.povUp();
  final Trigger armSetpointDown = operator.povDown();



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
