package frc.lib.controller;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ThrustmasterJoystick {
    private final Joystick joystick;

    private final Button trigger;
    private final Button bottomThumb;
    private final Button leftThumb;
    private final Button rightThumb;
    private final Button leftTopLeft;
    private final Button leftTopMiddle;
    private final Button leftTopRight;
    private final Button leftBottomRight;
    private final Button leftBottomMiddle;
    private final Button leftBottomLeft;
    private final Button rightTopRight;
    private final Button rightTopMiddle;
    private final Button rightTopLeft;
    private final Button rightBottomLeft;
    private final Button rightBottomMiddle;
    private final Button rightBottomRight;

    private final Axis xAxis;
    private final Axis yAxis;
    private final Axis zAxis;
    private final Axis sliderAxis;

    private HashMap<String, String> buttonPurposeHashMap = new HashMap<String,String>();

    /**
     * @param port The port the controller is on
     */
    public ThrustmasterJoystick(int port) {
        joystick = new Joystick(port);

        trigger = new JoystickButton(joystick, 1);
        bottomThumb = new JoystickButton(joystick, 2);
        leftThumb = new JoystickButton(joystick, 3);
        rightThumb = new JoystickButton(joystick, 4);
        leftTopLeft = new JoystickButton(joystick, 5);
        leftTopMiddle = new JoystickButton(joystick, 6);
        leftTopRight = new JoystickButton(joystick, 7);
        leftBottomRight = new JoystickButton(joystick, 8);
        leftBottomMiddle = new JoystickButton(joystick, 9);
        leftBottomLeft = new JoystickButton(joystick, 10);
        rightTopRight = new JoystickButton(joystick, 11);
        rightTopMiddle = new JoystickButton(joystick, 12);
        rightTopLeft = new JoystickButton(joystick, 13);
        rightBottomLeft = new JoystickButton(joystick, 14);
        rightBottomMiddle = new JoystickButton(joystick, 15);
        rightBottomRight = new JoystickButton(joystick, 16);

        xAxis = new JoystickAxis(joystick, 0);
        yAxis = new JoystickAxis(joystick, 1);
        zAxis = new JoystickAxis(joystick, 2);
        sliderAxis = new JoystickAxis(joystick, 3);
        sliderAxis.setInverted(true);
    }

    public Button getTrigger() {
        return trigger;
    }

    public Button getBottomThumb() {
        return bottomThumb;
    }

    public Button getLeftThumb() {
        return leftThumb;
    }

    public Button getRightThumb() {
        return rightThumb;
    }

    public Button getLeftTopLeft() {
        return leftTopLeft;
    }

    public Button getLeftTopMiddle() {
        return leftTopMiddle;
    }

    public Button getLeftTopRight() {
        return leftTopRight;
    }

    public Button getLeftBottomRight() {
        return leftBottomRight;
    }

    public Button getLeftBottomMiddle() {
        return leftBottomMiddle;
    }

    public Button getLeftBottomLeft() {
        return leftBottomLeft;
    }

    public Button getRightTopLeft() {
        return rightTopLeft;
    }

    public Button getRightTopMiddle() {
        return rightTopMiddle;
    }

    public Button getRightTopRight() {
        return rightTopRight;
    }

    public Button getRightBottomRight() {
        return rightBottomRight;
    }

    public Button getRightBottomMiddle() {
        return rightBottomMiddle;
    }

    public Button getRightBottomLeft() {
        return rightBottomLeft;
    }

    public Axis getXAxis() {
        return xAxis;
    }

    public Axis getYAxis() {
        return yAxis;
    }

    public Axis getZAxis() {
        return zAxis;
    }

    public Axis getSliderAxis() {
        return sliderAxis;
    }

    public void nameTrigger(String purpose) {
        buttonPurposeHashMap.put("trigger", purpose);
    }

    public void nameBottomThumb(String purpose) {
        buttonPurposeHashMap.put("bottomThumb", purpose);
    }

    public void nameLeftThumb(String purpose) {
        buttonPurposeHashMap.put("leftThumb", purpose);
    }

    public void nameRightThumb(String purpose) {
        buttonPurposeHashMap.put("rightThumb", purpose);
    }

    public void nameLeftTopLeft(String purpose) {
        buttonPurposeHashMap.put("leftTopLeft", purpose);
    }

    public void nameLeftTopMiddle(String purpose) {
        buttonPurposeHashMap.put("leftTopMiddle", purpose);
    }

    public void nameLeftTopRight(String purpose) {
        buttonPurposeHashMap.put("leftTopRight", purpose);
    }

    public void nameLeftBottomRight(String purpose) {
        buttonPurposeHashMap.put("leftBottomRight", purpose);
    }

    public void nameLeftBottomMiddle(String purpose) {
        buttonPurposeHashMap.put("leftBottomMiddle", purpose);
    }

    public void nameLeftBottomLeft(String purpose) {
        buttonPurposeHashMap.put("leftBottomLeft", purpose);
    }

    public void nameRightTopLeft(String purpose) {
        buttonPurposeHashMap.put("rightTopLeft", purpose);
    }

    public void nameRightTopMiddle(String purpose) {
        buttonPurposeHashMap.put("rightTopMiddle", purpose);
    }

    public void nameRightTopRight(String purpose) {
        buttonPurposeHashMap.put("rightTopRight", purpose);
    }

    public void nameRightBottomRight(String purpose) {
        buttonPurposeHashMap.put("rightBottomRight", purpose);
    }

    public void nameRightBottomMiddle(String purpose) {
        buttonPurposeHashMap.put("rightBottomMiddle", purpose);
    }

    public void nameRightBottomLeft(String purpose) {
        buttonPurposeHashMap.put("rightBottomLeft", purpose);
    }

    public void nameXAxis(String purpose) {
        buttonPurposeHashMap.put("xAxis", purpose);
    }

    public void nameYAxis(String purpose) {
        buttonPurposeHashMap.put("yAxis", purpose);
    }

    public void nameZAxis(String purpose) {
        buttonPurposeHashMap.put("zAxis", purpose);
    }

    public void nameSliderAxis(String purpose) {
        buttonPurposeHashMap.put("sliderAxis", purpose);
    }
}
