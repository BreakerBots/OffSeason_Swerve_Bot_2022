// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** A class that represents the set of four function or action buttons common to most gamepad controllers, 
 * examples being the ABXY buttons of Xbox controllers or the shape buttons of playstation controllers */
public class BreakerGamepadActionButtons {
    private JoystickButton topActionButton, leftActionButton, rightActionButton, bottomActonButton;
    public BreakerGamepadActionButtons(GenericHID hid, int topActionButtonPort, int leftActionButtonPort, int rightActionButtonPort, int bottomActionButtonPort) {
        topActionButton = new JoystickButton(hid, topActionButtonPort);
        leftActionButton = new JoystickButton(hid, leftActionButtonPort);
        rightActionButton = new JoystickButton(hid, rightActionButtonPort);
        bottomActonButton = new JoystickButton(hid, bottomActionButtonPort);
    }

    public JoystickButton getBottomActionButton() {
        return bottomActonButton;
    }

    public JoystickButton getLeftActionButton() {
        return leftActionButton;
    }

    public JoystickButton getRightActionButton() {
        return rightActionButton;
    }

    public JoystickButton getTopActionButton() {
        return topActionButton;
    }
}
