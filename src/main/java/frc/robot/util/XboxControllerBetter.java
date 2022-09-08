package frc.robot.util;

import com.team2539.cougarlib.controller.Axis;
import com.team2539.cougarlib.controller.JoystickAxis;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class XboxControllerBetter {
    private final Joystick joystick;

    private final Button A;
    private final Button B;
    private final Button X;
    private final Button Y;
    private final Button leftBumper;
    private final Button rightBumper;
    private final Button back;
    private final Button start;
    private final Button leftJoystick;
    private final Button rightJoystick;
    // private final Button dPadUp;
    // private final Button dPadRight;
    // private final Button dPadDown;
    // private final Button dPadLeft;

    private final Axis leftXAxis;
    private final Axis leftYAxis;
    private final Axis rightXAxis;
    private final Axis rightYAxis;
    // private final Axis dPadXAxis;
    // private final Axis dPadYAxis;
    private final Axis leftTrigger;
    private final Axis rightTrigger;

    /**
     * @param port The port the controller is on
     */
    public XboxControllerBetter(int port) {
        joystick = new Joystick(port);

        A = new JoystickButton(joystick, 1);
        B = new JoystickButton(joystick, 2);
        X = new JoystickButton(joystick, 3);
        Y = new JoystickButton(joystick, 4);
        leftBumper = new JoystickButton(joystick, 5);
        rightBumper = new JoystickButton(joystick, 6);
        back = new JoystickButton(joystick, 7);
        start = new JoystickButton(joystick, 8);
        
        leftJoystick = new JoystickButton(joystick, 9);
        rightJoystick = new JoystickButton(joystick, 10);

        leftXAxis = new JoystickAxis(joystick, 0);
        leftYAxis = new JoystickAxis(joystick, 1);
        leftTrigger = new JoystickAxis(joystick, 2);
        rightTrigger = new JoystickAxis(joystick, 3);
        rightXAxis = new JoystickAxis(joystick, 4);
        rightYAxis = new JoystickAxis(joystick, 5);
        
        leftYAxis.setInverted(true);
        rightYAxis.setInverted(true);
        // dPadYAxis.setInverted(true);
    }

    public Button getA() {
        return A;
    }

    public Button getB() {
        return B;
    }

    public Button getX() {
        return X;
    }

    public Button getY() {
        return Y;
    }

    public Button getLeftBumper() {
        return leftBumper;
    }

    public Button getRightBumper() {
        return rightBumper;
    }

    public Axis getLeftTrigger() {
        return leftTrigger;
    }

    public Axis getRightTrigger() {
        return rightTrigger;
    }

    public Button getBack() {
        return back;
    }

    public Button getStart() {
        return start;
    }

    public Button getLeftJoystick() {
        return leftJoystick;
    }

    public Button getRightJoystick() {
        return rightJoystick;
    }

    public Axis getLeftXAxis() {
        return leftXAxis;
    }

    public Axis getLeftYAxis() {
        return leftYAxis;
    }

    public Axis getRightXAxis() {
        return rightXAxis;
    }

    public Axis getRightYAxis() {
        return rightYAxis;
    }

}
