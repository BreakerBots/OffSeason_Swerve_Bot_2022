// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Interface for all Swerve Modules to allwo for easy interchangeablity, this class is meant to surve as an intermedairy between your swerve hardware and the BreakerSwerveDrive class */
public interface BreakerGenericSwerveModule extends BreakerGenericDevice {

    /** default method for setting a swerve module to a given target state, 
     * automaticly calls the overloded version of this method that independently specifyes angle and speed*/
    public default void setModuleTarget(SwerveModuleState targetModuleState) {
        setModuleTarget(targetModuleState.angle, targetModuleState.speedMetersPerSecond);
    }

    /** Method defined in module code to handle angle and velocity control of the module */
    public abstract void setModuleTarget(Rotation2d targetAngle, double targetVelocityMetersPerSecond);

    /** @return the absolute (+/- 180 deg) angle of the module in degrees*/
    public abstract double getModuleAbsoluteAngle();

    /** @return the relative (with rollover, 180 -> 181) angle of the module in degrees*/
    public abstract double getModuleRelativeAngle();
    
    /** @return the velocity of the module's drive wheel in meters per second*/
    public abstract double getModuleVelMetersPerSec();

    public abstract double getMetersPerSecToNativeVelUnits(double speedMetersPerSec);

    public abstract SwerveModuleState getModuleState();

    public abstract void setDriveMotorBrakeMode(boolean isEnabled);

    public abstract void setTurnMotorBreakMode(boolean isEnabled);

    public abstract void setModuleBreakMode(boolean isEnabled);

    /** returns the modules health as an array [0] = overall, [1] = drive motor, [2] = turn motor, [3] = outher if supported (EX: CANCoder)*/
    public abstract DeviceHealth[] getModuleHealths();
}
