package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {
    private Spark ledController = new Spark(Constants.LED_CONTROLLER_PWM_ID);

    private LightMode lightMode = LightMode.DEFAULT;

    private Supplier<Boolean> statusSupplier = null;

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

    public void blinkWhiteSlow() {
        set(-0.21);
    }

    public void blinkWhiteFast() {
        set(-0.04);
    }

    public void showTeamColor() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) solidBlue();
        else solidRed();
    }

    public void aiming() {
        if (statusSupplier != null) {
            if (statusSupplier.get()) solidGreen();
            else showTeamColor();
        }
    }

    public void balltrackFull() {
        if (statusSupplier != null) {
            if (statusSupplier.get()) blinkWhiteFast();
            else showTeamColor();
        }
    }

    public void setDefaultMode() {
        lightMode = LightMode.DEFAULT;
    }

    public void setAimingMode(Supplier<Boolean> statusSupplier) {
        lightMode = LightMode.AIMING;
        this.statusSupplier = statusSupplier;
    }

    public void setBalltrackMode(Supplier<Boolean> statusSupplier) {
        lightMode = LightMode.BALLTRACK_FULL;
        this.statusSupplier = statusSupplier;
    }

    @Override
    public void periodic() {
        switch (lightMode) {
            case DEFAULT:
                showTeamColor();
                break;
            case AIMING:
                aiming();
                break;
            case BALLTRACK_FULL:
                balltrackFull();
                break;
        }
    }

    private enum LightMode {
        DEFAULT,
        AIMING,
        BALLTRACK_FULL
    }
}
