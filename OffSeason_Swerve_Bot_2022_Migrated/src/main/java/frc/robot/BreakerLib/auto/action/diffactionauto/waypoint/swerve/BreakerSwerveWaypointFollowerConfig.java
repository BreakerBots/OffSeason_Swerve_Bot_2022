// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.action.diffactionauto.waypoint.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;

/** Add your docs here. */
public class BreakerSwerveWaypointFollowerConfig {
    private HolonomicDriveController driveController;
    private BreakerSwerveDrive drivetrain;
    private BreakerGenericOdometer odometer;

    public BreakerSwerveWaypointFollowerConfig(BreakerSwerveDrive drivetrain, HolonomicDriveController driveController) {
        this(drivetrain, drivetrain, driveController);
    }

    public BreakerSwerveWaypointFollowerConfig(BreakerSwerveDrive drivetrain, BreakerGenericOdometer odometer, HolonomicDriveController driveController) {
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
