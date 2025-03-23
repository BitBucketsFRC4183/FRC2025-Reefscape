package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorInput {
  public static final CommandXboxController operator = new CommandXboxController(1);
  public static final CommandXboxController driver = new CommandXboxController(0);


  // DRIVER'S CONTROLS

  // static bc im lazy
  public static final Trigger slowModeHold = driver.leftTrigger();
  public static final Trigger turboModeHold = driver.rightTrigger();
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
  public final Trigger resetHeading = driver.start();

  // dpad not pressed
  public static final Trigger alignmentRobotRelative = driver.povCenter().negate();
  //operator controls
//  final Trigger elevatorGoToOrigin = operator.a();
//  final Trigger elevatorSetpoint1 = operator.x();
//  final Trigger elevatorSetpoint2 = operator.y();
//  final Trigger elevatorSetpoint3 = operator.b();
  final Trigger manualElevator = operator.axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, 0.1);
  final Trigger resetElevatorEncoder = operator.start();

  final Trigger manualArm = operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.1);
  final Trigger manualPivot = operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.1);
  final Trigger intakeClaw = operator.leftTrigger();
  final Trigger outtakeClaw = operator.rightTrigger();
  final Trigger armElevatorOrigin = operator.a();
  final Trigger armElevatorL2 = operator.x();
  final Trigger armElevatorL3 = operator.y();
  final Trigger armElevatorL4 = operator.b();
  final Trigger rollersIn = operator.leftBumper();
  final Trigger rollersOut = operator.rightBumper();

  public static final Trigger mechanismLimitOverride = operator.back();
  final Trigger rollerPivotUp = operator.povUp();
  final Trigger rollerPivotDown = operator.povDown();



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
