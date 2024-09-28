package frc.robot.lib.input;

import static frc.robot.lib.input.ControllerConstants.XboxConstants.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Implementation of an Xbox controller
 */
public class XboxController implements Controller {
    private Joystick joystick; // Joystick object representing the Xbox controller

    // Trigger objects for each button on the Xbox controller
    private Trigger aButton, bButton, xButton, yButton, leftMiddleButton, rightMiddleButton, 
                   leftBumper, rightBumper, leftStick, rightStick, 
                   leftTriggerButton, rightTriggerButton, rightStickYButton, 
                   upDpad, downDpad, leftDpad, rightDpad;

    // Constructor that initializes the joystick with a given instance
    public XboxController(Joystick joystick) {
        this.joystick = joystick;
        this.mapController(); // Map the buttons to their respective triggers
    }

    // Constructor that initializes the joystick with a given port number
    public XboxController(int port) {
        this(new Joystick(port)); // Create a new Joystick instance using the specified port
    }

    @Override
    public void mapController() {
        // Map each button on the joystick to its corresponding trigger
        this.aButton = new JoystickButton(joystick, BUTTON_A);
        this.bButton = new JoystickButton(joystick, BUTTON_B);
        this.xButton = new JoystickButton(joystick, BUTTON_X);
        this.yButton = new JoystickButton(joystick, BUTTON_Y);
        this.leftMiddleButton = new JoystickButton(joystick, BUTTON_LEFT_MIDDLE);
        this.rightMiddleButton = new JoystickButton(joystick, BUTTON_RIGHT_MIDDLE);
        this.leftBumper = new JoystickButton(joystick, BUMPER_LEFT);
        this.rightBumper = new JoystickButton(joystick, BUMPER_RIGHT);
        this.leftStick = new JoystickButton(joystick, BUTTON_LEFT_STICK);
        this.rightStick = new JoystickButton(joystick, BUTTON_RIGHT_STICK);
        this.upDpad = new POVButton(joystick, 0); // Up D-pad button
        this.rightDpad = new POVButton(joystick, 90); // Right D-pad button
        this.downDpad = new POVButton(joystick, 180); // Down D-pad button
        this.leftDpad = new POVButton(joystick, 270); // Left D-pad button
    }

    /**
     * @return the value of the left trigger
     */
    public double getLeftTrigger() {
        return joystick.getRawAxis(JOYSTICK_LEFT_TRIGGER); // Get raw axis value for left trigger
    }

    /**
     * @return the value of the right trigger
     */
    public double getRightTrigger() {
        return joystick.getRawAxis(JOYSTICK_RIGHT_TRIGGER); // Get raw axis value for right trigger
    }

    /**
     * @return the value of the left joystick x-axis
     */
    public double getLeftX() {
        return joystick.getRawAxis(JOYSTICK_LEFT_X); // Get raw axis value for left joystick X-axis
    }

    /**
     * @return the value of the left joystick y-axis (up is positive and down is negative)
     */
    public double getLeftY() {
        return joystick.getRawAxis(JOYSTICK_LEFT_Y); // Get raw axis value for left joystick Y-axis
    }

    /**
     * @return the value of the right joystick x-axis
     */
    public double getRightX() {
        return joystick.getRawAxis(JOYSTICK_RIGHT_X); // Get raw axis value for right joystick X-axis
    }

    /**
     * @return the value of the right joystick y-axis (up is positive and down is negative)
     */
    public double getRightY() {
        return joystick.getRawAxis(JOYSTICK_RIGHT_Y); // Get raw axis value for right joystick Y-axis
    }

    /**
     * @return the A button trigger
     */
    public Trigger getAButton() {
        return aButton; // Return A button trigger
    }

    /**
     * @return the B button trigger
     */
    public Trigger getBButton() {
        return bButton; // Return B button trigger
    }

    /**
     * @return the X button trigger
     */
    public Trigger getXButton() {
        return xButton; // Return X button trigger
    }

    /**
     * @return the Y button trigger
     */
    public Trigger getYButton() {
        return yButton; // Return Y button trigger
    }

    /**
     * @return the left middle button (start button? view button?)
     */
    public Trigger getLeftMiddleButton() {
        return leftMiddleButton; // Return left middle button trigger
    }

    /**
     * @return the right middle button (select button? menu button?)
     */
    public Trigger getRightMiddleButton() {
        return rightMiddleButton; // Return right middle button trigger
    }

    /**
     * @return the left bumper trigger
     */
    public Trigger getLeftBumper() {
        return leftBumper; // Return left bumper trigger
    }

    /**
     * @return the right bumper trigger
     */
    public Trigger getRightBumper() {
        return rightBumper; // Return right bumper trigger
    }

    /**
     * @return the left stick as a trigger
     */
    public Trigger getLeftStick() {
        return leftStick; // Return left stick trigger
    }

    /**
     * @return the right stick as a trigger
     */
    public Trigger getRightStick() {
        return rightStick; // Return right stick trigger
    }

    /**
     * @return the right trigger as a button (not implemented in original code)
     */
    public Trigger getRightTriggerButton() {
        return rightTriggerButton; // Return right trigger as a button (if implemented)
    }

   /**
     * @return the left trigger as a button (not implemented in original code)
     */
   public Trigger getLeftTriggerButton() {
       return leftTriggerButton; // Return left trigger as a button (if implemented)
   }

   /**
     * @return the right stick's y-axis as a button (not implemented in original code)
     */
   public Trigger getRightStickYButton() {
       return rightStickYButton; // Return right stick's Y-axis as a button (if implemented)
   }

   /**
     * @return the up D-pad button trigger 
     */
   public Trigger getUpDpad() {
       return upDpad; // Return up D-pad button trigger 
   }

   /**
      * @return the down D-pad button trigger 
      */
   public Trigger getDownDpad() {
       return downDpad; // Return down D-pad button trigger 
   }

   /**
      * @return the left D-pad button trigger 
      */
   public Trigger getLeftDpad() {
       return leftDpad; // Return left D-pad button trigger 
   }

   /**
      * @return the right D-pad button trigger 
      */
   public Trigger getRightDpad() {
       return rightDpad; // Return right D-pad button trigger 
   }

   /**
      * Sets the rumble amount on the controller.
      *
      * @param type  Which rumble value to set (left or right motor)
      * @param value The normalized value (0 to 1) to set the rumble to.
      */
   public void setRumble(RumbleType type, double value) {
       joystick.setRumble(type, value); // Set rumble effect on specified motor with given intensity.
   }
}
