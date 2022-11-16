// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led.animations;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class BreakerRainbowAnimation extends SubsystemBase implements BreakerAnimation {
    private Map<Color, Integer> colorsAndIndexes;
    public BreakerRainbowAnimation(int length, int dencity, double speed) {

    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isActive() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public AddressableLEDBuffer getBuffer() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int getLength() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
    
}
