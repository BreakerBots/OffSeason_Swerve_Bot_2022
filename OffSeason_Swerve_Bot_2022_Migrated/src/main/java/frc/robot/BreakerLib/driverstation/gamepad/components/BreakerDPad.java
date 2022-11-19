// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class BreakerDPad {

    public static final class DefaultButtonAngles {
        public static final int UP = 0;
        public static final int TOP_RIGHT = 45;
        public static final int RIGHT = 90;
        public static final int BOTTOM_RIGHT = 135;
        public static final int DOWN = 180;
        public static final int BOTTOM_LEFT = 225;
        public static final int LEFT = 270;
        public static final int TOP_LEFT = 315;
    }

    private POVButton dPadUp;
    private POVButton dPadTopRight;
    private POVButton dPadDown;
    private POVButton dPadBottomRight;
    private POVButton dPadLeft;
    private POVButton dPadBottomLeft;
    private POVButton dPadRight;
    private POVButton dPadTopLeft;

    public BreakerDPad(GenericHID hid) {
        this(
            hid, 
            DefaultButtonAngles.UP,
            DefaultButtonAngles.TOP_RIGHT,
            DefaultButtonAngles.RIGHT, 
            DefaultButtonAngles.BOTTOM_RIGHT, 
            DefaultButtonAngles.DOWN, 
            DefaultButtonAngles.BOTTOM_LEFT,
            DefaultButtonAngles.BOTTOM_LEFT, 
            DefaultButtonAngles.TOP_LEFT
        );
    }
    
    
    public BreakerDPad(
        GenericHID hid, 
        int upButtonAng, 
        int topRightButtonAng, 
        int rightButtonAng, 
        int bottomRightButtonAng, 
        int downButtonAng, 
        int bottomLeftButtonAng, 
        int leftButtonAng, 
        int topLeftButtonAng
        ) {
        dPadUp = new POVButton(hid, upButtonAng);
        dPadTopRight = new POVButton(hid, topRightButtonAng);
        dPadDown = new POVButton(hid, downButtonAng);
        dPadBottomRight = new POVButton(hid, bottomRightButtonAng);
        dPadLeft = new POVButton(hid, leftButtonAng);
        dPadBottomLeft = new POVButton(hid, bottomLeftButtonAng);
        dPadRight = new POVButton(hid, rightButtonAng);
        dPadTopLeft = new POVButton(hid, topLeftButtonAng);
    }

    public POVButton getDown() {
        return dPadDown;
    }

    public POVButton getLeft() {
        return dPadLeft;
    }

    public POVButton getRight() {
        return dPadRight;
    }

    public POVButton getUp() {
        return dPadUp;
    }

    public POVButton getTopLeft() {
        return dPadTopLeft;
    }

    public POVButton getTopRight() {
        return dPadTopRight;
    }

    public POVButton getBottomLeft() {
        return dPadBottomLeft;
    }

    public POVButton getBottomRight() {
        return dPadBottomRight;
    }

    
}
