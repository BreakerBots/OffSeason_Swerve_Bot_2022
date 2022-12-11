// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.devices.motors.binary;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import BreakerLib.util.power.BreakerPowerManagementConfig;
import BreakerLib.util.power.DevicePowerMode;
import BreakerLib.util.test.selftest.DeviceHealth;
import BreakerLib.util.vendor.BreakerCTREUtil;
import edu.wpi.first.math.Pair;

/** Falcon motor with simple on/off controls */
public class BreakerBinaryCTREMotor extends BreakerGenericBinaryMotor {

    private BaseMotorController motor;
    private double output;

    /**
     * Create a new BinaryTalon that switches between 100% forward output and 0%
     * output.
     * 
     * @param motor CTRE motor controller.
     */
    public BreakerBinaryCTREMotor(BaseMotorController motor) {
        this.motor = motor;
        this.output = 1.0;
        deviceName = " Binary_Motor (" + motor.getDeviceID() + ") ";

    }

    /**
     * Create a new BinaryTalon that switches between given output % and 0%
     * output.
     * 
     * @param motor  CTRE motor controller.
     * @param output Percent output between -1 and 1.
     */
    public BreakerBinaryCTREMotor(BaseMotorController motor, double output) {
        this.motor = motor;
        this.output = output;
        deviceName = " Binary_Motor (" + motor.getDeviceID() + ") ";
    }

    @Override
    /** Sets motor to designated percent output. */
    public void start() {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    /** Sets motor to 0% output (stopped) */
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    /** Checks if motor is running or not. */
    public boolean isActive() {
        return (motor.getMotorOutputPercent() != 0);
    }

    /** @return Base CTRE motor controller. */
    public BaseMotorController getMotor() {
        return motor;
    }

    @Override
    public void toggle() {
        if (isActive()) {
            stop();
        } else {
            start();
        }
    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;
        Faults faultObj = new Faults();
        motor.getFaults(faultObj);
        if (faultObj.hasAnyFault()) {
            Pair<DeviceHealth, String> pair = BreakerCTREUtil
                    .getMotorHealthAndFaults(faultObj);
            faultStr = pair.getSecond();
            health = pair.getFirst();
        }
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
    public boolean isUnderAutomaticControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public DevicePowerMode getPowerMode() {
        // TODO Auto-generated method stub
        return null;
    }

}
