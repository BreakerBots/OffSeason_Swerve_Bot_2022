// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class BreakerGamepadThumbstick extends BreakerAnalogThumbstick{
    private JoystickButton button;
    public BreakerGamepadThumbstick(GenericHID hid, int thumbstickButtonPort, int xAxisPort, boolean invertX, int yAxisPort, boolean invertY) {
        super(hid, xAxisPort, invertX, yAxisPort, invertY);
        button = new JoystickButton(hid, thumbstickButtonPort);
    }

    public BreakerGamepadThumbstick(GenericHID hid, int thumbstickButtonPort, int xAxisPort, int yAxisPort) {
        super(hid, xAxisPort, yAxisPort);
        button = new JoystickButton(hid, thumbstickButtonPort);
    }

    public JoystickButton getJoystickButton() {
        return button;
    }
}
