// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led.animations;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Add your docs here. */
public class BreakerColorSwitchAnimation extends CommandBase implements BreakerAnimation {
    private int length, curColorIndex = 0;
    private double timePerColorSeconds;
    private boolean isStoped = true;
    private AddressableLEDBuffer buff;
    private Color8Bit[] switchColors;
    private final Timer timer = new Timer();
    public BreakerColorSwitchAnimation(int length, double timePerColorSeconds, Color... colors) {
        this.length = length;
        this.timePerColorSeconds = timePerColorSeconds;
        switchColors = new Color8Bit[colors.length];
        for (int i = 0; i < colors.length; i++) {
            switchColors[i] = new Color8Bit(colors[i]);
        }
        buff = new AddressableLEDBuffer(length);
        for (int i = 0; i < length; i++) {
            buff.setLED(i, switchColors[0]);
        }
    }

    @Override
    public void start() {
        isStoped = false;
        this.schedule();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public void stop() {
       isStoped = true;
    }

    @Override
    public boolean isActive() {
        return !isStoped;
    }

    @Override
    public AddressableLEDBuffer getBuffer() {
        return buff;
    }

    @Override
    public int getLength() {
        return length;
    }

    @Override
    public void execute() {
        if (timer.get() >= timePerColorSeconds) {
            timer.reset();
            curColorIndex++;
            for (int i = 0; i < length; i++) {
                buff.setLED(i, switchColors[curColorIndex]);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isStoped;
    }
    
}
