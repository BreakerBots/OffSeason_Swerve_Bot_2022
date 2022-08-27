// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.devices.BreakerGenericLoopedDevice;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;

/** Contianer class for methods common to all drivetrain types */
public abstract class BreakerGenericDrivetrain extends BreakerGenericLoopedDevice implements BreakerGenericOdometer {
    protected boolean slowModeActive = false;

    public  void setSlowMode(boolean isEnabled) {
        slowModeActive = isEnabled;
    }

    public boolean isInSlowMode() {
        return slowModeActive;
    }

    /** Updates the odometer position. */
    public abstract void updateOdometry();

    public abstract void setDrivetrainBrakeMode(boolean isEnabled);

    public abstract ChassisSpeeds getFieldRelativeChassisSpeeds(BreakerGenericOdometer odometer);

}
