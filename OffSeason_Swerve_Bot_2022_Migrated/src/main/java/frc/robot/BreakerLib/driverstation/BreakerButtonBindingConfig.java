// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class BreakerButtonBindingConfig {
    public enum BreakerButtonBindingType {
        ON_TRUE,
        ON_FALSE,
        WHILE_TRUE,
        WHILE_FALSE,
        TOGGLE_ON_TRUE,
        TOGGLE_ON_FALSE,
        WHEN_ACTIVE,
        WHILE_ACTIVE_CONTINUIOUS,
        WHILE_ACTIVE_ONCE,
        WHEN_INACTIVE,
        TOGGLE_WHEN_ACTIVE,
        CANCLE_WHEN_ACTIVE
    }

    public BreakerButtonBindingConfig(Trigger bindingTrigger, BreakerButtonBindingType bindingType, Command commandToBind) {
        
    }
}
