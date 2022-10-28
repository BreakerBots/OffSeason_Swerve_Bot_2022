// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.devices.motors.BreakerSmartMotorController.SmartMotorControlMode;

/** Add your docs here. */
public interface BreakerGenericSmartMotorController extends MotorController, BreakerGenericDevice {
    public enum SmartMotorControlMode {
        VOLTAGE,
        PRECENT,
        POSITION,
        VELOCITY,
        CURRENT
    }

    public default void set(double speed) {
        set(SmartMotorControlMode.PRECENT, speed);
    }

    public default void set(SmartMotorControlMode controlMode, double value) {
        set(controlMode, value, false, 0.0);
    }

    public abstract void set(SmartMotorControlMode controlMode, double value, boolean useArbFF, double arbFF);

    public abstract void setPIDSlot(int slot);

    /** ticks */
    public abstract double getSelectedSensorPosition();

    /** ticks per sec */
    public abstract double getSelectedSensorVelocity();

    /** ticks per sec */
    public abstract double getSelectedSensorVelocityRPM();
}
