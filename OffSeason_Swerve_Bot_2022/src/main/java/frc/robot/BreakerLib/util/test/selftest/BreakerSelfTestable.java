// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.selftest;

/** Interface for all devises capable of automated self testing */
public interface BreakerSelfTestable {
    /** A metod called pereodicly by the {@link SelfTest} class, 
     * sets device helth and fault string when called*/
    public abstract void runSelfTest();

    /** Should return the {@link DeviceHealth}.NOMINAL value unless a device fault or error is present */
    public abstract DeviceHealth getHealth();

    public abstract String getFaults();

    public abstract String getDeviceName();

    public abstract boolean hasFault();

    public abstract void setDeviceName(String newName);
}
