package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {
    private Spark ledController = new Spark(Constants.LED_CONTROLLER_PWM_ID);

    public LightsSubsystem() {}

    public void set(double pulseWidth) {
        ledController.set(pulseWidth);
    }

    public void off() {
        set(0.99);
    }

    public void solidRed() {
        set(0.61);
    }

    public void solidGreen() {
        set(0.71);
    }

    public void solidYellow() {
        set(0.69);
    }

    public void solidBlue() {
        set(0.83);
    }

    public void solidOrange() {
        set(0.63);
    }

    public void solidWhite() {
        set(0.93);
    }

    public void solidPink() {
        set(0.57);
    }

    public void solidPurple() {
        set(0.91);
    }

    public void fire() {
        set(-0.57);
    }

    public void chase() {
        set(-0.31);
    }

    public void blinkWhite() {
        set(-0.21);
    }

    public void showTeamColor() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) solidBlue();
        else solidRed();
    }
}
