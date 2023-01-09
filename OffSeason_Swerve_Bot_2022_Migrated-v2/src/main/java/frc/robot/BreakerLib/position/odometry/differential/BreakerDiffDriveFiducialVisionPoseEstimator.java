// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.differential;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;

/** Estimates diff drive pose based on vision odometry. */
public class BreakerDiffDriveFiducialVisionPoseEstimator implements BreakerGenericOdometer {
    private BreakerVisionOdometer vision;
    private BreakerDiffDrivePoseEstimationOdometer poseEstimator;

    public BreakerDiffDriveFiducialVisionPoseEstimator(BreakerDiffDrive drivetrain, BreakerVisionOdometer vision,
            double[] stateStanderdDeveation, double[] encoderAndGyroStandardDeveation,
            double[] visionStanderdDeveation) {
        this.vision = vision;
        poseEstimator = new BreakerDiffDrivePoseEstimationOdometer(drivetrain, vision.getOdometryPoseMeters(),
                stateStanderdDeveation, visionStanderdDeveation);
        
        CommandScheduler.getInstance().schedule(new RunCommand(() -> updateOdometry()));
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        vision.setOdometryPosition(newPose);
        poseEstimator.setOdometryPosition(newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return poseEstimator.getOdometryPoseMeters();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return poseEstimator.getMovementState();
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return poseEstimator.getRobotRelativeChassisSpeeds();
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return poseEstimator.getFieldRelativeChassisSpeeds();
    }

    private void updateOdometry() {
        if (vision.isAnyTargetVisable()) {
            poseEstimator.addVisionMeasurment(vision.getOdometryPoseMeters(), vision.getDataTimestamp());
        }
    }
}
