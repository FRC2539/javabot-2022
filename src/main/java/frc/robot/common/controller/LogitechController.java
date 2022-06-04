package frc.robot.common.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class LogitechController {
    private final Joystick joystick;

    private final Button A;
    private final Button B;
    private final Button X;
    private final Button Y;
    private final Button leftBumper;
    private final Button rightBumper;
    private final Button leftTrigger;
    private final Button rightTrigger;
    private final Button back;
    private final Button start;
    private final Button leftJoystick;
    private final Button rightJoystick;
    private final Button dPadUp;
    private final Button dPadRight;
    private final Button dPadDown;
    private final Button dPadLeft;

    private final Axis leftXAxis;
    private final Axis leftYAxis;
    private final Axis rightXAxis;
    private final Axis rightYAxis;
    private final Axis dPadXAxis;
    private final Axis dPadYAxis;

    /**
     * @param port The port the controller is on
     */
    public LogitechController(int port) {
        joystick = new Joystick(port);

        A = new JoystickButton(joystick, 2);
        B = new JoystickButton(joystick, 3);
        X = new JoystickButton(joystick, 1);
        Y = new JoystickButton(joystick, 4);
        leftBumper = new JoystickButton(joystick, 5);
        rightBumper = new JoystickButton(joystick, 6);
        leftTrigger = new JoystickButton(joystick, 7);
        rightTrigger = new JoystickButton(joystick, 8);
        back = new JoystickButton(joystick, 9);
        start = new JoystickButton(joystick, 10);
        leftJoystick = new JoystickButton(joystick, 11);
        rightJoystick = new JoystickButton(joystick, 12);
        dPadUp = new JoystickButton(joystick, 20);
        dPadRight = new JoystickButton(joystick, 21);
        dPadDown = new JoystickButton(joystick, 22);
        dPadLeft = new JoystickButton(joystick, 23);

        leftXAxis = new JoystickAxis(joystick, 0);
        leftYAxis = new JoystickAxis(joystick, 1);
        rightXAxis = new JoystickAxis(joystick, 2);
        rightYAxis = new JoystickAxis(joystick, 3);
        dPadXAxis = new JoystickAxis(joystick, 4);
        dPadYAxis = new JoystickAxis(joystick, 5);
        leftYAxis.setInverted(true);
        rightYAxis.setInverted(true);
        dPadYAxis.setInverted(true);
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

    public Button getLeftTrigger() {
        return leftTrigger;
    }

    public Button getRightTrigger() {
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

    public Button getDPadUp() {
        return dPadUp;
    }

    public Button getDPadRight() {
        return dPadRight;
    }

    public Button getDPadDown() {
        return dPadDown;
    }

    public Button getDPadLeft() {
        return dPadLeft;
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

    public Axis getDPadXAxis() {
        return dPadXAxis;
    }

    public Axis getDPadYAxis() {
        return dPadYAxis;
    }
}
