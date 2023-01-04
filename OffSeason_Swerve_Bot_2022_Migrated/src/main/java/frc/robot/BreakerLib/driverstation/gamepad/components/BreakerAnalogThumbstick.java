// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;

/** Class which represents an analog HID thumbstick. */
public class BreakerAnalogThumbstick {
    private GenericHID hid;
    private int xAxisPort, yAxisPort;
    private boolean invertX, invertY;
    private double xDeadband = 0.0, yDeadband = 0.0;
    

    /**
     * Constructs an analog thumbstick with desired inverts.
     * 
     * @param hid Controller.
     * @param xAxisPort X-axis port #.
     * @param yAxisPort Y-axis port #.
     */
    public BreakerAnalogThumbstick(GenericHID hid, int xAxisPort, int yAxisPort) {
        this(hid, xAxisPort, false, yAxisPort, false);
    }

    /**
     * Constructs an analog thumbstick with desired inverts.
     * 
     * @param hid Controller.
     * @param xAxisPort X-axis port #.
     * @param invertX Invert X-axis.
     * @param yAxisPort Y-axis port #.
     * @param invertY Invert Y-axis.
     */
    public BreakerAnalogThumbstick(GenericHID hid, int xAxisPort, boolean invertX, int yAxisPort, boolean invertY) {
        this.hid = hid;
        this.xAxisPort = xAxisPort;
        this.yAxisPort = yAxisPort;
        this.invertX = invertX;
        this.invertY = invertY;
    }

    /** Set stick deadbands.
     * 
     * @param xDeadband Deadband for x-axis.
     * @param yDeadband Deadband for y-axis.
     */
    public void setDeadband(double xDeadband, double yDeadband) {
        this.xDeadband = xDeadband;
        this.yDeadband = yDeadband;
    }

    /** @return Raw X-axis value. */
    public double getRawX() {
        return hid.getRawAxis(xAxisPort);
    }

    /** @return Raw Y-axis value. */
    public double getRawY() {
        return hid.getRawAxis(yAxisPort);
    }

    /** @return X-axis value. */
    public double getX() {
        return MathUtil.applyDeadband(getRawX(), xDeadband) * (invertX ? -1 : 1);
    }

    /** @return Y-axis value. */
    public double getY() {
        return MathUtil.applyDeadband(getRawY(), yDeadband) * (invertY ? -1 : 1);
    }

    /** @return If stick inputs outside of the deadband are detected. */
    public boolean isActive() {
        return (getY() != 0 || getX() !=0);
    }
}
