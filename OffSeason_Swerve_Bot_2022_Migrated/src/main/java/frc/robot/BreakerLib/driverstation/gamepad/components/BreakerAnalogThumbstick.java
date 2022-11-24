// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;

/** Add your docs here. */
public class BreakerAnalogThumbstick {
    private GenericHID hid;
    private int xAxisPort, yAxisPort;
    private boolean invertX, invertY;
    private double xDeadband = 0.0, yDeadband = 0.0;
    public BreakerAnalogThumbstick(GenericHID hid, int xAxisPort, int yAxisPort) {
        this(hid, xAxisPort, false, yAxisPort, false);
    }

    public BreakerAnalogThumbstick(GenericHID hid, int xAxisPort, boolean invertX, int yAxisPort, boolean invertY) {
        this.hid = hid;
        this.xAxisPort = xAxisPort;
        this.yAxisPort = yAxisPort;
        this.invertX = invertX;
        this.invertY = invertY;
    }

    public void setDeadband(double xDeadband, double yDeadband) {
        this.xDeadband = xDeadband;
        this.yDeadband = yDeadband;
    }

    public double getRawX() {
        return hid.getRawAxis(xAxisPort);
    }

    public double getRawY() {
        return hid.getRawAxis(yAxisPort);
    }

    public double getX() {
        return MathUtil.applyDeadband(getRawX(), xDeadband) * (invertX ? -1 : 1);
    }

    public double getY() {
        return MathUtil.applyDeadband(getRawY(), yDeadband) * (invertY ? -1 : 1);
    }
}
