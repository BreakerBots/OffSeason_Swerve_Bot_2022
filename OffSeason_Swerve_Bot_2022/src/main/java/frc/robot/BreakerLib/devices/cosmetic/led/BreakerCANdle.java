// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.devices.BreakerGenericLoopedDevice;
import frc.robot.BreakerLib.util.BreakerCTREUtil;
import frc.robot.BreakerLib.util.BreakerTriplet;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.selftest.SelfTest;

/** CTRE LED controller */
public class BreakerCANdle extends BreakerGenericLoopedDevice {

    public enum BreakerCANdleLEDMode {
        COLOR_SWITCH,
        ANIMATION,
        STATIC,
        ERROR,
        ENABLED,
        DISSABLED,
        OFF
    }

    private CANdle candle;
    private RainbowAnimation enabledStatus;
    private StrobeAnimation errorStatus;
    private int colorSwitch = 0;
    private int timer = 0;
    private int canID;
    private double switchTimeSec;
    private Color[] switchColors;
    private BreakerCANdleLEDMode mode = BreakerCANdleLEDMode.OFF;

    public BreakerCANdle(int canID, int numberOfLEDs, BreakerCANdleConfig config) {
        candle = new CANdle(canID);
        this.canID = canID;
        candle.configAllSettings(config.getConfig());
        candle.setLEDs(255, 255, 255);
        enabledStatus = new RainbowAnimation(1, 0.5, numberOfLEDs);
        errorStatus = new StrobeAnimation(255, 0, 0, 0, 0.5, numberOfLEDs);
        deviceName = " CANdle_LED_Controller ("+ canID +") ";
    }

    public void setLEDAnimation(Animation animation) {
        candle.animate(animation);
        mode = BreakerCANdleLEDMode.ANIMATION;
    }

    public void setRobotEnabledStatusLED() {
        mode = BreakerCANdleLEDMode.ENABLED;
    }

    public void setLEDOff() {
        mode = BreakerCANdleLEDMode.OFF;
    }

    public void runErrorStatusLED() {
        mode = BreakerCANdleLEDMode.ERROR;
    }

    public void setStaticLED(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
        mode = BreakerCANdleLEDMode.STATIC;
    }

    public void setStaticLED(Color ledColor) {
        candle.setLEDs(colorToRGB(ledColor)[0], colorToRGB(ledColor)[1], colorToRGB(ledColor)[2]);
        mode = BreakerCANdleLEDMode.STATIC;
    }

    private void setLED(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }

    private int[] colorToRGB(Color color) {
        return new int[] { (int) (color.red * 255), (int) (color.blue * 255), (int) (color.green * 255) };
    }

    private void setLED(Color ledColor) {
        candle.setLEDs(colorToRGB(ledColor)[0], colorToRGB(ledColor)[1], colorToRGB(ledColor)[2]);
    }

    public void setLEDColorSwitch(double switchTimeSec, Color... switchColors) {
        this.switchTimeSec = switchTimeSec;
        this.switchColors = switchColors;
        colorSwitch = 0;
        mode = BreakerCANdleLEDMode.COLOR_SWITCH;
    }

    private void runColorSwitch(double switchTimeSec, Color... colors) {
        if (timer % (switchTimeSec * 50) == 0) {
            colorSwitch = (colorSwitch > colors.length) ? 0 : colorSwitch;
            setLED(colors[colorSwitch++]);
        }
    }

    private void runLED() {
        switch (mode) {
            default:
            case OFF:
                setLED(0, 0, 0);
                break;
            case COLOR_SWITCH:
                runColorSwitch(switchTimeSec, switchColors);
                break;
            case ERROR:
                candle.animate(errorStatus);
                break;
            case ENABLED:
                candle.animate(enabledStatus);
                break;
            case DISSABLED:
                setLED(Color.kOrange);
                break;
            case ANIMATION:
            case STATIC:
                break;
        }
    }

    @Override
    public void periodic() {
        timer++;
        runLED();
    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;
        CANdleFaults faultsC = new CANdleFaults();
        candle.getFaults(faultsC);
        if (faultsC.hasAnyFault() || SelfTest.checkIsMissingCanID(canID)) {
            BreakerTriplet<DeviceHealth, String, Boolean> faultData = BreakerCTREUtil.getCANdelHealthFaultsAndConnectionStatus(faultsC, canID);
            faultStr = faultData.getMiddle();
            health = faultData.getLeft();
        } else {
            health = DeviceHealth.NOMINAL;
        }
        
    }

    @Override
    public boolean isUnderAutomaticControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public DevicePowerMode getPowerMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void returnToAutomaticPowerManagement() {
        // TODO Auto-generated method stub
        
    }
}
