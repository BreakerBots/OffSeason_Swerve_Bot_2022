// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.position.odometry.differential;

import BreakerLib.position.movement.BreakerMovementState2d;
import BreakerLib.position.odometry.BreakerGenericOdometer;
import BreakerLib.position.odometry.vision.BreakerVisionOdometer;
import BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

/** Estimates diff drive pose based on vision odometry. */
public class BreakerDiffDriveFiducialVisionPoseEstimator implements BreakerGenericOdometer {
    private BreakerVisionOdometer vision;
    private BreakerDiffDrive drivetrain;
    private BreakerDiffDrivePoseEstimationOdometer poseEstimator;

    public BreakerDiffDriveFiducialVisionPoseEstimator(BreakerDiffDrive drivetrain, BreakerVisionOdometer vision,
            double[] stateModelStanderdDeveation, double[] encoderAndGyroStandardDeveation,
            double[] visionStanderdDeveation) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        poseEstimator = new BreakerDiffDrivePoseEstimationOdometer(drivetrain.getBaseGyro(), vision.getOdometryPoseMeters(),
                stateModelStanderdDeveation, encoderAndGyroStandardDeveation, visionStanderdDeveation);
        
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
        poseEstimator.update(drivetrain.getDiffDriveState());
        if (vision.isAnyTargetVisable()) {
            poseEstimator.addVisionMeasurment(vision.getOdometryPoseMeters(), vision.getDataTimestamp());
        }
    }
}
