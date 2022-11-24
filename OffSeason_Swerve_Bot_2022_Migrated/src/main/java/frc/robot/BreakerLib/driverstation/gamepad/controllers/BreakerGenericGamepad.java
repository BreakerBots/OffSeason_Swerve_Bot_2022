package frc.robot.BreakerLib.driverstation.gamepad.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BreakerLib.driverstation.gamepad.BreakerControllerRumbleType;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerAnalogTrigger;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerDPad;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerFaceButtons;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogDeadbandConfig;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadThumbstick;

public class BreakerGenericGamepad {
    protected GenericHID hid;
    protected BreakerFaceButtons faceButtons;
    protected BreakerGamepadThumbstick leftJoystick, rightJoystick;
    protected BreakerAnalogTrigger leftTrigger, rightTrigger;
    protected JoystickButton leftBumper, rightBumper;
    protected BreakerDPad dPad;

    public BreakerGenericGamepad(
            GenericHID hid,
            int topActionButtonPort,
            int leftActionButtonPort,
            int rightActionButtonPort,
            int bottomActionButtonPort,
            int leftThumbstickButtonPort,
            int leftThumbstickAxisPortX,
            boolean invertLeftThumbstickAxisPortX,
            int leftThumbstickAxisPortY,
            boolean invertLeftThumbstickAxisPortY,
            int rightThumbstickButtonPort,
            int rightThumbstickAxisPortX,
            boolean invertRightThumbstickAxisPortX,
            int rightThumbstickAxisPortY,
            boolean invertRightThumbstickAxisPortY,
            int leftTriggerAxisPort,
            int rightTriggerAxisPort,
            int leftBumperPort,
            int rightBumperPort) {
        this.hid = hid;
        faceButtons = new BreakerFaceButtons(hid, topActionButtonPort, leftActionButtonPort,
                rightActionButtonPort, bottomActionButtonPort);
        leftJoystick = new BreakerGamepadThumbstick(hid, leftThumbstickButtonPort, leftThumbstickAxisPortX,
                invertLeftThumbstickAxisPortX, leftThumbstickAxisPortY, invertLeftThumbstickAxisPortY);
        rightJoystick = new BreakerGamepadThumbstick(hid, rightThumbstickButtonPort, rightThumbstickAxisPortX,
                invertRightThumbstickAxisPortX, rightThumbstickAxisPortY, invertRightThumbstickAxisPortY);
        leftTrigger = new BreakerAnalogTrigger(hid, leftTriggerAxisPort);
        rightTrigger = new BreakerAnalogTrigger(hid, rightTriggerAxisPort);
        leftBumper = new JoystickButton(hid, leftBumperPort);
        rightBumper = new JoystickButton(hid, rightBumperPort);
        dPad = new BreakerDPad(hid);
    }

    public void configDeadbands(BreakerGamepadAnalogDeadbandConfig deadbandConfig) {
        leftJoystick.setDeadband(deadbandConfig.getLeftX(), deadbandConfig.getLeftY());
        rightJoystick.setDeadband(deadbandConfig.getRightX(), deadbandConfig.getRightY());
        leftTrigger.setDeadband(deadbandConfig.getLeftTriggerAxis());
        rightTrigger.setDeadband(deadbandConfig.getRightTriggerAxis());
    }

    public void setRumble(BreakerControllerRumbleType rumbleType, double rumblePrecent) {
        switch (rumbleType) {
            case COARSE:
                hid.setRumble(RumbleType.kLeftRumble, rumblePrecent);
                break;
            case FINE:
                hid.setRumble(RumbleType.kRightRumble, rumblePrecent);
                break;
            case MIXED:
                hid.setRumble(RumbleType.kLeftRumble, rumblePrecent);
                hid.setRumble(RumbleType.kRightRumble, rumblePrecent);
                break;
            default:
                hid.setRumble(RumbleType.kLeftRumble, rumblePrecent);
                hid.setRumble(RumbleType.kRightRumble, rumblePrecent);
                break;
        }
    }

    public void setMixedRumble(double leftRumble, double rightRumble) {
        hid.setRumble(RumbleType.kLeftRumble, leftRumble);
        hid.setRumble(RumbleType.kRightRumble, leftRumble);
    }

    public void clearRumble() {
        setMixedRumble(0, 0);
    }

    public BreakerFaceButtons getActionButtons() {
        return faceButtons;
    }

    public BreakerGamepadThumbstick getLeftThumbstick() {
        return leftJoystick;
    }

    public BreakerGamepadThumbstick getRightThumbstick() {
        return rightJoystick;
    }

    public BreakerAnalogTrigger getLeftTrigger() {
        return leftTrigger;
    }

    public BreakerAnalogTrigger getRightTrigger() {
        return rightTrigger;
    }

    public BreakerDPad getDPad() {
        return dPad;
    }

    public JoystickButton getLeftBumper() {
        return leftBumper;
    }

    public JoystickButton getRightBumper() {
        return rightBumper;
    }

    public GenericHID getBaseHID() {
        return hid;
    }

}