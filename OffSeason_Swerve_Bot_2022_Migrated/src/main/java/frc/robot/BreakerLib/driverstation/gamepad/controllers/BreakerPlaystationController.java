// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.controllers;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Playstation duleshock/dulesence controller wrapper. */
public class BreakerPlaystationController extends BreakerGenericGamepad {
    private JoystickButton triangle, square, circle, cross, share, options, touchpad, ps;

    public BreakerPlaystationController(int controllerPortNum) {
        super(new PS4Controller(controllerPortNum), PS4Controller.Button.kTriangle.value,
                PS4Controller.Button.kSquare.value, PS4Controller.Button.kCircle.value,
                PS4Controller.Button.kCross.value,
                PS4Controller.Button.kL3.value, PS4Controller.Axis.kLeftX.value, false, PS4Controller.Axis.kLeftY.value,
                false, PS4Controller.Button.kR3.value, PS4Controller.Axis.kRightX.value, false,
                PS4Controller.Axis.kRightY.value, false, PS4Controller.Axis.kL2.value, PS4Controller.Axis.kR2.value,
                PS4Controller.Button.kL1.value,
                PS4Controller.Button.kL1.value);
                
        triangle = new JoystickButton(hid, PS4Controller.Button.kTriangle.value);
        square = new JoystickButton(hid, PS4Controller.Button.kSquare.value);
        circle = new JoystickButton(hid, PS4Controller.Button.kCircle.value);
        cross = new JoystickButton(hid, PS4Controller.Button.kCross.value);
        share = new JoystickButton(hid, PS4Controller.Button.kShare.value);
        options = new JoystickButton(hid, PS4Controller.Button.kOptions.value);
        touchpad = new JoystickButton(hid, PS4Controller.Button.kTouchpad.value);
        ps = new JoystickButton(hid, PS4Controller.Button.kPS.value);
    }

    public JoystickButton getTriangleButton() {
        return triangle;
    }

    public JoystickButton getSquareButton() {
        return square;
    }

    public JoystickButton getCircleButton() {
        return circle;
    }

    public JoystickButton getCrossButton() {
        return cross;
    }

    public JoystickButton getShareButton() {
        return share;
    }

    public JoystickButton getOptionsButton() {
        return options;
    }

    public JoystickButton getPlayStationButton() {
        return ps;
    }

    public JoystickButton getTouchpadButton() {
        return touchpad;
    }

}
