/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.OIConstants;

/**
 * Handle input from a Dualshock controller connected to the Driver Station.
 */
public class DualShockController extends GenericHID {
  /**
   * Represents a digital button on a DualShockController.
   */
  public enum Button {
    kRect(1), kCross(2), kDisk(3), kTrig(4), kBumperLeft(5), kBumperRight(6), kTriggerLeft(7), kTriggerRight(8),
    kShare(9), kOption(10), kStickLeft(11), kStickRight(12), kPad(14);

    @SuppressWarnings({ "MemberName", "PMD.SingularField" })
    public final int value;

    Button(int value) {
      this.value = value;
    }
  }

  private double[] controllerMapping;
  private boolean mappingInitialized = false;

  /**
   * Construct an instance of a joystick. The joystick index is the USB port on
   * the drivers station.
   *
   * @param port The port on the Driver Station that the joystick is plugged into.
   */
  public DualShockController(final int port) {
    super(port);
  }

  /**
   * @param curvature >0, recommanded below 5, larger number = more curve. The
   *                  controller curve follows this function
   *                  https://www.desmos.com/calculator/p2q98lfjet
   */
  public void initMapping(double curvature) {
    controllerMapping = new double[OIConstants.controllerPrecision + 5];
    double tempX, w2, w1 = Math.exp(curvature * (-0.1));
    for (int i = 0; i < controllerMapping.length; i++) {
      // mapping i (probally 0 to 1000) to -1 to 1
      tempX = (i - (OIConstants.controllerPrecision / 2.0)) / (OIConstants.controllerPrecision / 2.0);
      w2 = w1 + Math.exp(10.0 * (Math.abs(tempX) - 1)) * (1 - w1);
      controllerMapping[i] = tempX * w2;
    }
    mappingInitialized = true;
  }

  /**
   * if controller mapping not initialized, the input is returned
   * 
   * @param input , -1<=input<=1, it's recommanded to do deadzone first
   * @return controller input on a curve
   */
  private double getMappedOutput(double input) {
    if (!mappingInitialized)
      return input;
    return controllerMapping[(int) Math
        .round(input * (OIConstants.controllerPrecision / 2) + (OIConstants.controllerPrecision / 2))];
  }

  /**
   * @param input , just dump the output of getRawAxis(#) directly in here,
   *              -1<=input<=1
   * @return controller input adjusted to deadzone
   */
  private double getDeadzonedOutput(double input) {
    // https://www.desmos.com/calculator/oubwvzj81f
    return Math.abs(input) < OIConstants.joystickDeadzone ? 0
        : (input - Math.signum(input) * OIConstants.joystickDeadzone) / (1 - OIConstants.joystickDeadzone);
  }

  public double getXMapped(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getMappedOutput(getDeadzonedOutput(getRawAxis(0)));
    } else {
      return getMappedOutput(getDeadzonedOutput(getRawAxis(2)));
    }
  }

  public double getYMapped(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getMappedOutput(getDeadzonedOutput(getRawAxis(1)));
    } else {
      return getMappedOutput(getDeadzonedOutput(getRawAxis(5)));
    }
  }

  // ******************************************
  // * Stuff copied from XboxController Below *
  // ******************************************

  /**
   * Get the X axis value of the controller.
   *
   * @param hand Side of controller whose value should be returned.
   * @return The X axis value of the controller.
   */
  @Override
  public double getX(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getDeadzonedOutput(getRawAxis(0));
    } else {
      return getDeadzonedOutput(getRawAxis(2));
    }
  }

  /**
   * Get the Y axis value of the controller.
   *
   * @param hand Side of controller whose value should be returned.
   * @return The Y axis value of the controller.
   */
  @Override
  public double getY(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getDeadzonedOutput(getRawAxis(1));
    } else {
      return getDeadzonedOutput(getRawAxis(5));
    }
  }

  /**
   * Get the trigger axis value of the controller.
   *
   * @param hand Side of controller whose value should be returned.
   * @return The trigger axis value of the controller.
   */
  public double getTriggerAxis(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getDeadzonedOutput(getRawAxis(3));
    } else {
      return getDeadzonedOutput(getRawAxis(4));
    }
  }

  /**
   * Read the value of the bumper button on the controller.
   *
   * @param hand Side of controller whose value should be returned.
   * @return The state of the button.
   */
  public boolean getBumper(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getRawButton(Button.kBumperLeft.value);
    } else {
      return getRawButton(Button.kBumperRight.value);
    }
  }

  /**
   * Whether the bumper was pressed since the last check.
   *
   * @param hand Side of controller whose value should be returned.
   * @return Whether the button was pressed since the last check.
   */
  public boolean getBumperPressed(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonPressed(Button.kBumperLeft.value);
    } else {
      return getRawButtonPressed(Button.kBumperRight.value);
    }
  }

  /**
   * Whether the bumper was released since the last check.
   *
   * @param hand Side of controller whose value should be returned.
   * @return Whether the button was released since the last check.
   */
  public boolean getBumperReleased(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonReleased(Button.kBumperLeft.value);
    } else {
      return getRawButtonReleased(Button.kBumperRight.value);
    }
  }

  /**
   * Read the value of the stick button on the controller.
   *
   * @param hand Side of controller whose value should be returned.
   * @return The state of the button.
   */
  public boolean getStickButton(Hand hand) {
    if (hand.equals(Hand.kLeft)) {
      return getRawButton(Button.kStickLeft.value);
    } else {
      return getRawButton(Button.kStickRight.value);
    }
  }

  /**
   * Whether the stick button was pressed since the last check.
   *
   * @param hand Side of controller whose value should be returned.
   * @return Whether the button was pressed since the last check.
   */
  public boolean getStickButtonPressed(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonPressed(Button.kStickLeft.value);
    } else {
      return getRawButtonPressed(Button.kStickRight.value);
    }
  }

  /**
   * Whether the stick button was released since the last check.
   *
   * @param hand Side of controller whose value should be returned.
   * @return Whether the button was released since the last check.
   */
  public boolean getStickButtonReleased(Hand hand) {
    if (hand == Hand.kLeft) {
      return getRawButtonReleased(Button.kStickLeft.value);
    } else {
      return getRawButtonReleased(Button.kStickRight.value);
    }
  }

  /**
   * Read the value of the square button on the controller.
   *
   * @return The state of the square button.
   */
  public boolean getRectButton() {
    return getRawButton(Button.kRect.value);
  }

  /**
   * Whether the square button was pressed since the last check.
   *
   * @return Whether the square button was pressed since the last check.
   */
  public boolean getRectButtonPressed() {
    return getRawButtonPressed(Button.kRect.value);
  }

  /**
   * Whether the square button was released since the last check.
   *
   * @return Whether the square button was released since the last check.
   */
  public boolean getRectButtonReleased() {
    return getRawButtonReleased(Button.kRect.value);
  }

  /**
   * Read the value of the cross button on the controller.
   *
   * @return The state of the cross button.
   */
  public boolean getCrossButton() {
    return getRawButton(Button.kCross.value);
  }

  /**
   * Whether the cross button was pressed since the last check.
   *
   * @return Whether the cross button was pressed since the last check.
   */
  public boolean getCrossButtonPressed() {
    return getRawButtonPressed(Button.kCross.value);
  }

  /**
   * Whether the cross button was released since the last check.
   *
   * @return Whether the cross button was released since the last check.
   */
  public boolean getCrossButtonReleased() {
    return getRawButtonReleased(Button.kCross.value);
  }

  /**
   * Read the value of the circle button on the controller.
   *
   * @return The state of the circle button.
   */
  public boolean getDiskButton() {
    return getRawButton(Button.kDisk.value);
  }

  /**
   * Whether the circle button was pressed since the last check.
   *
   * @return Whether the circle button was pressed since the last check.
   */
  public boolean getDiskButtonPressed() {
    return getRawButtonPressed(Button.kDisk.value);
  }

  /**
   * Whether the circle button was released since the last check.
   *
   * @return Whether the circle button was released since the last check.
   */
  public boolean getDiskButtonReleased() {
    return getRawButtonReleased(Button.kDisk.value);
  }

  /**
   * Read the value of the triangle button on the controller.
   *
   * @return The state of the triangle button.
   */
  public boolean getTrigButton() {
    return getRawButton(Button.kTrig.value);
  }

  /**
   * Whether the triangle button was pressed since the last check.
   *
   * @return Whether the triangle button was pressed since the last check.
   */
  public boolean getTrigButtonPressed() {
    return getRawButtonPressed(Button.kTrig.value);
  }

  public boolean getTrigHeld() {
    return getRawButton(Button.kTrig.value);
  }

  /**
   * Whether the triangle button was released since the last check.
   *
   * @return Whether the triangle button was released since the last check.
   */
  public boolean getTrigButtonReleased() {
    return getRawButtonReleased(Button.kTrig.value);
  }

  /**
   * Read the value of the share button on the controller.
   *
   * @return The state of the share button.
   */
  public boolean getShareButton() {
    return getRawButton(Button.kShare.value);
  }

  /**
   * Whether the share button was pressed since the last check.
   *
   * @return Whether the share button was pressed since the last check.
   */
  public boolean getShareButtonPressed() {
    return getRawButtonPressed(Button.kShare.value);
  }

  /**
   * Whether the share button was released since the last check.
   *
   * @return Whether the share button was released since the last check.
   */
  public boolean getShareButtonReleased() {
    return getRawButtonReleased(Button.kShare.value);
  }

  /**
   * Read the value of the option button on the controller.
   *
   * @return The state of the option button.
   */
  public boolean getOptionButton() {
    return getRawButton(Button.kOption.value);
  }

  /**
   * Whether the option button was pressed since the last check.
   *
   * @return Whether the option button was pressed since the last check.
   */
  public boolean getOptionButtonPressed() {
    return getRawButtonPressed(Button.kOption.value);
  }

  /**
   * Whether the option button was released since the last check.
   *
   * @return Whether the option button was released since the last check.
   */
  public boolean getOptionButtonReleased() {
    return getRawButtonReleased(Button.kOption.value);
  }

  public boolean getTriggerLeftButton(){
    return getRawButton(Button.kTriggerLeft.value);
  }

  public boolean getTriggerRightButton(){
    return getRawButton(Button.kTriggerRight.value);
  }

public Object getTriggerLeftPressed() {
	return getRawButtonPressed(Button.kTriggerLeft.value);
}
}
