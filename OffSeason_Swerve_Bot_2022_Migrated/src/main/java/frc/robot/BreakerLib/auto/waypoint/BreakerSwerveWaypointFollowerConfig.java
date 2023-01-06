// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import edu.wpi.first.math.controller.HolonomicDriveController;

import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;

/** Config for {@link BreakerSwerveWaypointFollower} with holonomic drive controller, drivetrain, and odometer. */
public class BreakerSwerveWaypointFollowerConfig {

    private HolonomicDriveController driveController;
    private BreakerSwerveDrive drivetrain;
    private BreakerGenericOdometer odometer;

    /** Creates config with internal drivetrain odometer.
     * 
     * @param drivetrain Swerve drive.
     * @param driveController Holonomic drive controller.
     */
    public BreakerSwerveWaypointFollowerConfig(BreakerSwerveDrive drivetrain, HolonomicDriveController driveController) {
        this(drivetrain, driveController, drivetrain);
    }

    /** Creates config with external odometer.
     * 
     * @param drivetrain Swerve drive.
     * @param driveController Holonomic drive controller.
     * @param odometer External odometer.
     */
    public BreakerSwerveWaypointFollowerConfig(BreakerSwerveDrive drivetrain, HolonomicDriveController driveController, BreakerGenericOdometer odometer) {
        this.driveController = driveController;
        this.drivetrain = drivetrain;
        this.odometer = odometer;
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
