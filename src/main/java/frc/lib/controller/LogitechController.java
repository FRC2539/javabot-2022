package frc.lib.controller;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

public class LogitechController {
    private int port;

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

    private HashMap<String, String> buttonPurposeHashMap = new HashMap<String, String>();

    /**
     * @param port The port the controller is on
     */
    public LogitechController(int port) {
        this.port = port;

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

        buttonPurposeHashMap.put("type", "LogitechController");
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

    public void nameA(String purpose) {
        buttonPurposeHashMap.put("A", purpose);
    }

    public void nameB(String purpose) {
        buttonPurposeHashMap.put("B", purpose);
    }

    public void nameX(String purpose) {
        buttonPurposeHashMap.put("X", purpose);
    }

    public void nameY(String purpose) {
        buttonPurposeHashMap.put("Y", purpose);
    }

    public void nameLeftBumper(String purpose) {
        buttonPurposeHashMap.put("leftBumper", purpose);
    }

    public void nameRightBumper(String purpose) {
        buttonPurposeHashMap.put("rightBumper", purpose);
    }

    public void nameLeftTrigger(String purpose) {
        buttonPurposeHashMap.put("leftTrigger", purpose);
    }

    public void nameRightTrigger(String purpose) {
        buttonPurposeHashMap.put("rightTrigger", purpose);
    }

    public void nameBack(String purpose) {
        buttonPurposeHashMap.put("back", purpose);
    }

    public void nameStart(String purpose) {
        buttonPurposeHashMap.put("start", purpose);
    }

    public void nameLeftJoystick(String purpose) {
        buttonPurposeHashMap.put("leftJoystick", purpose);
    }

    public void nameRightJoystick(String purpose) {
        buttonPurposeHashMap.put("rightJoystick", purpose);
    }

    public void nameDPadUp(String purpose) {
        buttonPurposeHashMap.put("dPadUp", purpose);
    }

    public void nameDPadRight(String purpose) {
        buttonPurposeHashMap.put("dPadRight", purpose);
    }

    public void nameDPadDown(String purpose) {
        buttonPurposeHashMap.put("dPadDown", purpose);
    }

    public void nameDPadLeft(String purpose) {
        buttonPurposeHashMap.put("dPadLeft", purpose);
    }

    public void nameLftXAxis(String purpose) {
        buttonPurposeHashMap.put("leftXAxis", purpose);
    }

    public void nameLftYAxis(String purpose) {
        buttonPurposeHashMap.put("leftYAxis", purpose);
    }

    public void nameRghtXAxis(String purpose) {
        buttonPurposeHashMap.put("rightXAxis", purpose);
    }

    public void nameRghtYAxis(String purpose) {
        buttonPurposeHashMap.put("rightYAxis", purpose);
    }

    public void nameDadXAxis(String purpose) {
        buttonPurposeHashMap.put("dPadXAxis", purpose);
    }

    public void nameDadYAxis(String purpose) {
        buttonPurposeHashMap.put("dPadYAxis", purpose);
    }

    public void sendButtonNamesToNT() {
        NetworkTableInstance.getDefault()
                .getTable("Controllers")
                .getEntry(port + "")
                .setString(toJSON());
    }

    /**
     * @return Button names as a JSON String
     */
    public String toJSON() {
        return buttonPurposeHashMap.entrySet().stream()
                .map((Map.Entry<String, String> buttonEntry) -> stringifyButtonName(buttonEntry))
                .collect(Collectors.joining(", ", "{", "}"));
    }

    private String stringifyButtonName(Map.Entry<String, String> buttonEntry) {
        return "\"" + buttonEntry.getKey() + "\": " + "\"" + buttonEntry.getValue() + "\"";
    }
}
