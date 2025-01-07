package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorInput {
  final CommandXboxController operatorControl = new CommandXboxController(1);
  public final CommandXboxController driver = new CommandXboxController(0);

  public boolean actuallyIsTeleop = false;
  final Trigger isTeleop = new Trigger(() -> actuallyIsTeleop); // TODO fill this out

  // DRIVER'S CONTROLS
  public final Trigger slowModeHold = driver.leftTrigger();
  public final Trigger turboModeHold = driver.rightTrigger();

  final Trigger resetGyroPress = driver.start();

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

  public double getDriverRightComponentRaw() {
    return -driver.getLeftX(); // reference frame stuff
  }

  public double getRobotForwardComponentRaw() {
    return -driver.getLeftY(); // reference frame stuff
  }

  public double getRobotRotationRaw() {
    return driver.getRightX();
  }

  public Rotation2d getDriverRightAsAngle() {
    double rotZeroToOne = (driver.getRightX() + 1) % 1;

    return Rotation2d.fromRotations(rotZeroToOne);
  }

  public double getDriverAngularComponentRaw() {
    return deadband(-driver.getRightX());
  }
}
