// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;

/** Add your docs here. */
public class BreakerAnalogueTrigger {
    private GenericHID hid;
    private int port;
    private double deadband = 0.0;
    private boolean invert;
    public BreakerAnalogueTrigger(GenericHID hid, int analogTriggerAxisPort) {
        this.hid = hid;
        port = analogTriggerAxisPort;
        invert = false;
    }

    public BreakerAnalogueTrigger(GenericHID hid, int analogTriggerAxisPort, boolean invert) {
        this.hid = hid;
        port = analogTriggerAxisPort;
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    public double getRaw() {
        return hid.getRawAxis(port);
    }

    public double get() {
        return MathUtil.applyDeadband(port, deadband) * (invert ? -1 : 1);
    }
}
