// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class BreakerPWMLed {
    private AddressableLED led;
    private AddressableLEDBuffer buff;
    public BreakerPWMLed(int portPWM, int stripLength) {
        led = new AddressableLED(portPWM);
        led.setLength(stripLength);
        buff = new AddressableLEDBuffer(stripLength);
    }
}
