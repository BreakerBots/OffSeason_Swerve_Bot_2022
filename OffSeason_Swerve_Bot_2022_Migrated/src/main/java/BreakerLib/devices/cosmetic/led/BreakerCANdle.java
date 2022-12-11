// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.devices.cosmetic.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleFaults;

import BreakerLib.devices.BreakerGenericLoopedDevice;
import BreakerLib.devices.cosmetic.led.animations.BreakerAnimation;
import BreakerLib.util.BreakerRoboRIO.RobotOperatingMode;
import BreakerLib.util.power.BreakerPowerManagementConfig;
import BreakerLib.util.power.DevicePowerMode;
import BreakerLib.util.test.selftest.DeviceHealth;
import BreakerLib.util.vendor.BreakerCTREUtil;
import edu.wpi.first.math.Pair;

/** CTRE LED controller */
public class BreakerCANdle extends BreakerGenericLoopedDevice implements BreakerGenericLED {

    private CANdle candle;
    private boolean isOn = false, isActiveBAnim;

    /**
     * Creates a BreakerCANdle on the default CANBus.
     * 
     * @param canID  ID for CANdle.
     * @param config CANdle config
     */
    public BreakerCANdle(int canID, BreakerCANdleConfig config) {
        candle = new CANdle(canID);
        candle.configAllSettings(config.getConfig());
        deviceName = " CANdle_LED_Controller (" + canID + ") ";
    }

    /**
     * Creates a BreakerCANdle on named CANBus.
     * 
     * @param canID   ID for CANdle.
     * @param config  CANdle config
     * @param busName Name of CANBus
     */
    public BreakerCANdle(int canID, BreakerCANdleConfig config, String busName) {
        candle = new CANdle(canID, busName);
        candle.configAllSettings(config.getConfig());
        deviceName = " CANdle_LED_Controller (" + canID + ") ";
    }

    /**
     * Sets LEDs to play CTRE LED animation.
     * 
     * @param animation CANdle animation.
     */
    public void setCTREAnimation(Animation animation) {
        candle.animate(animation);
    }

    @Override
    public void setAllLEDs(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    @Override
    public void setLEDsInRange(int r, int g, int b, int startIndex, int endIndex) {
        candle.setLEDs(r, g, b, 0, startIndex, endIndex - startIndex);

    }

    @Override
    public void setLED(int r, int g, int b, int index) {
        candle.setLEDs(r, g, b, 0, index, 1);

    }

    @Override
    public void setAnimation(BreakerAnimation state) {

    }

    @Override
    public void setAnimation(int startIndex, BreakerAnimation state) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setAnimation(int startIndex, int endIndex, BreakerAnimation state) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setRobotModeDefaultState(RobotOperatingMode mode, BreakerAnimation state) {
        // TODO Auto-generated method stub

    }

    @Override
    public void clearToModeDefault() {
        // TODO Auto-generated method stub

    }

    @Override
    public void clearActiveAnimation() {
        // TODO Auto-generated method stub

    }

    @Override
    public void setOn() {
        isOn = true;
    }

    @Override
    public void setOff() {
        isOn = false;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public boolean isInModeDefault() {
        return isActiveBAnim;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;
        CANdleFaults faultsC = new CANdleFaults();
        candle.getFaults(faultsC);
        if (faultsC.hasAnyFault()) {
            Pair<DeviceHealth, String> faultData = BreakerCTREUtil.getCANdleHealthAndFaults(faultsC);
            faultStr = faultData.getSecond();
            health = faultData.getFirst();
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
