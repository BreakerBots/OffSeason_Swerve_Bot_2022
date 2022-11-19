// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.trinarymotor;

import frc.robot.BreakerLib.devices.BreakerGenericDeviceBase;

/** Add your docs here. */
public abstract class BreakerGenericTrinaryMotor extends BreakerGenericDeviceBase {
        /** Sets motor to designated forward percent output. */
        public abstract void forward();
    
        /** Sets motor to designated reverse percent output. */
        public abstract void reverse();
    
        /** Sets motor to 0% output (stopped) */
        public abstract void stop();
    
        /** Checks if motor is running or not. */
        public abstract boolean isActive();
}
