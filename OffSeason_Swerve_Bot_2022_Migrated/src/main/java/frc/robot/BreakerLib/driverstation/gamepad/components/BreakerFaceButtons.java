// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A class that represents the face button set common to most gamepad
 * controllers,
 * ex. ABXY buttons of Xbox controllers or the shape buttons of PlayStation
 * controllers.
 */
public class BreakerFaceButtons {
    private JoystickButton topActionButton, leftActionButton, rightActionButton, bottomActonButton;

    /** Constructs set of face button based on given button ports and HID device. */
    public BreakerFaceButtons(GenericHID hid, int topActionButtonPort, int leftActionButtonPort,
            int rightActionButtonPort, int bottomActionButtonPort) {
        topActionButton = new JoystickButton(hid, topActionButtonPort);
        leftActionButton = new JoystickButton(hid, leftActionButtonPort);
        rightActionButton = new JoystickButton(hid, rightActionButtonPort);
        bottomActonButton = new JoystickButton(hid, bottomActionButtonPort);
    }

    /** @return Bottom face button (ex. A on Xbox). */
    public JoystickButton getBottomActionButton() {
        return bottomActonButton;
    }

    /** @return Left face button (ex. X on Xbox). */
    public JoystickButton getLeftActionButton() {
        return leftActionButton;
    }

    /** @return Right face button (ex. B on Xbox). */
    public JoystickButton getRightActionButton() {
        return rightActionButton;
    }

    /** @return Top face button (ex. Y on Xbox). */
    public JoystickButton getTopActionButton() {
        return topActionButton;
    }
}
