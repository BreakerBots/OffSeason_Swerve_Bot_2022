// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.position.odometry.swerve;

import java.util.function.Supplier;

import BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import BreakerLib.position.movement.BreakerMovementState2d;
import BreakerLib.position.odometry.BreakerGenericOdometer;
import BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * BreakerLib wrapper for WPILib's {@link SwerveDriveOdometry}, which uses
 * swerve drive encoders and gyro yaw.
 */
public class BreakerSwerveEncoderOdometer implements BreakerGenericOdometer {

    SwerveDriveOdometry odometer;
    BreakerSwerveDrive drivetrain;
    BreakerGenericGyro gyro;
    SwerveDriveKinematics kinematics;

    public BreakerSwerveEncoderOdometer(BreakerSwerveDrive drivetrain, BreakerGenericGyro gyro) {
        kinematics = drivetrain.getConfig().getKinematics();
        this.gyro = gyro;
        odometer = new SwerveDriveOdometry(kinematics, this.gyro.getYawRotation2d(), drivetrain.getSwerveModulePositions());
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        odometer.resetPosition(newPose, gyro.getYawRotation2d(), drivetrain.getSwerveModulePositions());
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return odometer.getPoseMeters();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }
}
