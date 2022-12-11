// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.auto.trajectory.swerve.standard;

import BreakerLib.position.odometry.BreakerGenericOdometer;
import BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import edu.wpi.first.math.controller.HolonomicDriveController;

/** A config class that represents the inital setup for a {@link BreakerSwerveAutoPathFollower} */
public class BreakerSwerveAutoPathFollowerConfig {

    private BreakerSwerveDrive drivetrain;
    private HolonomicDriveController driveController;
    private BreakerGenericOdometer odometer;

    /**
     * Config with internal drivetrain odometer.
     * 
     * @param drivetrain Drivetrain
     * @param driveController Holonomic controller.
     */
    public BreakerSwerveAutoPathFollowerConfig(BreakerSwerveDrive drivetrain, HolonomicDriveController driveController) {
        this(drivetrain, driveController, drivetrain);
    }

    /**
     * Config with external odometer.
     * 
     * @param drivetrain Drivetrain
     * @param driveController Holonomic controller.
     * @param odometer External odometer.
     */
    public BreakerSwerveAutoPathFollowerConfig(BreakerSwerveDrive drivetrain, HolonomicDriveController driveController, BreakerGenericOdometer odometer) {
        this.drivetrain = drivetrain;
        this.odometer = odometer;
        this.driveController = driveController;
    }

    public HolonomicDriveController getDriveController() {
        return driveController;
    }

    public BreakerSwerveDrive getDrivetrain() {
        return drivetrain;
    }
    public BreakerGenericOdometer getOdometer() {
        return odometer;
    }
}
