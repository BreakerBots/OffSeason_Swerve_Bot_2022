package frc.robot.BreakerLib.driverstation.gamepad.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BreakerLib.driverstation.gamepad.BreakerControllerRumbleType;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerAnalogueTrigger;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerDPad;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadActionButtons;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogueDeadbandConfig;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadJoystick;

public class BreakerGenericGamepad {
    protected GenericHID hid;
    protected BreakerGamepadActionButtons actionButtons;
    protected BreakerGamepadJoystick leftJoystick, rightJoystick;
    protected BreakerAnalogueTrigger leftTrigger, rightTrigger;
    protected JoystickButton leftBumper, rightBumper;
    protected BreakerDPad dPad;
    public BreakerGenericGamepad(
            GenericHID hid,
            int topActionButtonPort, 
            int leftActionButtonPort, 
            int rightActionButtonPort, 
            int bottomActionButtonPort, 
            int leftJoystickButtonPort,
            int leftJoystickAxisPortX, 
            boolean invertLeftJoystickAxisPortX, 
            int leftJoystickAxisPortY, 
            boolean invertLeftJoystickAxisPortY, 
            int rightJoystickButtonPort,
            int rightJoystickAxisPortX, 
            boolean invertRightJoystickAxisPortX, 
            int rightJoystickAxisPortY, 
            boolean invertRightJoystickAxisPortY,
            int leftTriggerAxisPort,
            int rightTriggerAxisPort,
            int leftBumperPort,
            int rightBumperPort
            ) {
        this.hid = hid;
        actionButtons = new BreakerGamepadActionButtons(hid, topActionButtonPort, leftActionButtonPort, rightActionButtonPort, bottomActionButtonPort);
        leftJoystick = new BreakerGamepadJoystick(hid, leftJoystickButtonPort, leftJoystickAxisPortX, invertLeftJoystickAxisPortX, leftJoystickAxisPortY, invertLeftJoystickAxisPortY);
        rightJoystick = new BreakerGamepadJoystick(hid, rightJoystickButtonPort, rightJoystickAxisPortX, invertRightJoystickAxisPortX, rightJoystickAxisPortY, invertRightJoystickAxisPortY);
        leftTrigger = new BreakerAnalogueTrigger(hid, leftTriggerAxisPort);
        rightTrigger = new BreakerAnalogueTrigger(hid, rightTriggerAxisPort);
        leftBumper = new JoystickButton(hid, leftBumperPort);
        rightBumper = new JoystickButton(hid, rightBumperPort);
        dPad = new BreakerDPad(hid);
    }

    public void configDeadbands(BreakerGamepadAnalogueDeadbandConfig deadbandConfig) {
        leftJoystick.setDeadband(deadbandConfig.getLeftX(), deadbandConfig.getLeftY());
        rightJoystick.setDeadband(deadbandConfig.getRightX(), deadbandConfig.getRightY());
        leftTrigger.setDeadband(deadbandConfig.getLeftTriggerAxis());
        rightTrigger.setDeadband(deadbandConfig.getRightTriggerAxis());
    }

    public void setRumble(BreakerControllerRumbleType rumbleType, double rumblePrecent) {
        switch(rumbleType) {
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

    public BreakerGamepadActionButtons getActionButtons() {
        return actionButtons;
    }

    public BreakerGamepadJoystick getLeftJoystick() {
        return leftJoystick;
    }

    public BreakerGamepadJoystick getRightJoystick() {
        return rightJoystick;
    }

    public BreakerAnalogueTrigger getLeftTrigger() {
        return leftTrigger;
    }

    public BreakerAnalogueTrigger getRightTrigger() {
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