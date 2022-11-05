// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;

/** Add your docs here. */
public class BreakerSwerveAutoPathFollowerConfig {
    private BreakerSwerveDrive drivetrain;
    private HolonomicDriveController driveController;
    private BreakerGenericOdometer odometer;
    public BreakerSwerveAutoPathFollowerConfig(BreakerSwerveDrive drivetrain, HolonomicDriveController driveController, Pose2d tolerance) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        driveController.setTolerance(tolerance);
        odometer = drivetrain;
    }

    public BreakerSwerveAutoPathFollowerConfig(BreakerSwerveDrive drivetrain, BreakerGenericOdometer odometer, HolonomicDriveController driveController, Pose2d tolerance) {
        this.drivetrain = drivetrain;
        this.odometer = odometer;
        this.driveController = driveController;
        driveController.setTolerance(tolerance);
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
