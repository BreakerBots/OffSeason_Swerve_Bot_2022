// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.selftest;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.cscore.CameraServerCvJNI.Helper;
import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.BreakerTriplet;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerCTREUtil;

/** A higher level object for use in user susystems that makes BreakerLib's self test clases easier to implament for subsystem-scale classes */
public class SystemDiagnostics extends BreakerSelfTestableBase {
    private List<BaseMotorController> motorControllers = new ArrayList<>();
    private List<BreakerSelfTestable> devices = new ArrayList<>();
    private Supplier<DeviceHealth> deviceHealthSupplier;
    private Supplier<String> faultStringSupplier;
    private boolean usesSuppliers = false;
    public SystemDiagnostics(String systemName) {
        deviceName = systemName;
        usesSuppliers = false;
    }
 
    public void addSuppliers(Supplier<DeviceHealth> deviceHealthSupplier, Supplier<String> faultStringSupplier) {
        this.deviceHealthSupplier = deviceHealthSupplier;
        this.faultStringSupplier = faultStringSupplier;
        usesSuppliers = true;
    }

    public void addBreakerDevice(BreakerSelfTestable deviceToAdd) {
        devices.add(deviceToAdd);
    }

    public void addBreakerDevices(BreakerSelfTestable... devicesToAdd) {
        for (BreakerSelfTestable div: devicesToAdd) {
            devices.add(div);
        }
    }

    public void addMotorController(BaseMotorController motorControllerToAdd) {
        motorControllers.add(motorControllerToAdd);
    }

    public void addMotorControllers(BaseMotorController... motorControllersToAdd) {
        for (BaseMotorController con: motorControllersToAdd) {
            addMotorController(con);
        }
    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;
        if (!devices.isEmpty()) {
            for (BreakerSelfTestable div: devices) {
                div.runSelfTest();
                if (div.hasFault()) {
                    faultStr += " / " + div.getDeviceName() + ": " + div.getFaults();
                    health = health == DeviceHealth.INOPERABLE ? div.getHealth() : health;
                }
            }
        }
        if (!motorControllers.isEmpty()) {
            for (BaseMotorController con: motorControllers) {
                Faults motorFaults = new Faults();
                con.getFaults(motorFaults);
                if (motorFaults.hasAnyFault()) {
                    Pair<DeviceHealth, String> motorState = BreakerCTREUtil.getMotorHealthAndFaults(motorFaults);
                    faultStr += " / Motor ID (" + con.getBaseID() + "): " + motorState.getSecond();
                    health = health == DeviceHealth.INOPERABLE ? motorState.getFirst() : health;
                }
            }
        }
        if (usesSuppliers) {
            faultStr += " / Supplied Faults: " + faultStringSupplier.get();
            if (health != DeviceHealth.INOPERABLE && deviceHealthSupplier.get() != DeviceHealth.NOMINAL) {
                health = deviceHealthSupplier.get();
            }
        }
    }

}
