// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.BreakerLib.devices.BreakerGenericLoopedDevice;
import frc.robot.BreakerLib.devices.cosmetic.led.animations.BreakerAnimation;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotMode;
import frc.robot.BreakerLib.util.BreakerTriplet;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.selftest.SelfTest;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerCTREUtil;

/** CTRE LED controller */
public class BreakerCANdle extends BreakerGenericLoopedDevice implements BreakerGenericLEDDriver{

    private CANdle candle;
    private int canID;
    private boolean isOn = false, isActiveBAnim, isDefault;

    public BreakerCANdle(int canID, int numberOfLEDs, BreakerCANdleConfig config) {
        candle = new CANdle(canID);
        this.canID = canID;
        candle.configAllSettings(config.getConfig());
        deviceName = " CANdle_LED_Controller ("+ canID +") ";
    }

    public void setCTREAnimation(Animation animation) {
        candle.animate(animation);
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
    public void setRobotModeDefaultState(RobotMode mode, BreakerAnimation state) {
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
}
