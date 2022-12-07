// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.differential;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerDiffDrivePoseEstimator implements BreakerGenericOdometer {

    private BreakerGenericGyro gyro;
    private DifferentialDrivePoseEstimator poseEstimator;
    private Pose2d currentPose;
    private double lastUpdateTimestamp = Timer.getFPGATimestamp();
    private Pose2d prevPose = getOdometryPoseMeters();
    private ChassisSpeeds fieldRelativeChassisSpeeds = new ChassisSpeeds();
    private BreakerMovementState2d prevMovementState = new BreakerMovementState2d();
    private BreakerMovementState2d curMovementState = new BreakerMovementState2d();

    public BreakerDiffDrivePoseEstimator(BreakerGenericGyro gyro, Pose2d initialPose,
            double[] stateModelStanderdDeviation, double[] encoderAndGyroStandardDeviation,
            double[] visionStanderdDeviation) {
        currentPose = initialPose;
        this.gyro = gyro;
        poseEstimator = new DifferentialDrivePoseEstimator(Rotation2d.fromDegrees(gyro.getRawYaw()), initialPose,
                new MatBuilder<>(Nat.N5(), Nat.N1()).fill(stateModelStanderdDeviation[0],
                        stateModelStanderdDeviation[1], stateModelStanderdDeviation[2], stateModelStanderdDeviation[3],
                        stateModelStanderdDeviation[4]), // State measurement standard deviations. X, Y, theta.,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(encoderAndGyroStandardDeviation[0],
                        encoderAndGyroStandardDeviation[1], encoderAndGyroStandardDeviation[2]), // Local measurement
                                                                                                 // standard deviations.
                                                                                                 // Left encoder, right
                                                                                                 // encoder, gyro.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStanderdDeviation[0], visionStanderdDeviation[1],
                        visionStanderdDeviation[2])); // Global measurement standard deviations. X, Y, and theta.4
    }

    public Pose2d update(BreakerDiffDriveState currentDriveState) {
        prevPose = getOdometryPoseMeters();
        currentPose = poseEstimator.update(Rotation2d.fromDegrees(gyro.getRawYaw()), currentDriveState.getWheelSpeeds(),
                currentDriveState.getLeftDriveDistanceMeters(), currentDriveState.getRightDriveDistanceMeters());
        updateChassisSpeeds();
        lastUpdateTimestamp = Timer.getFPGATimestamp();
        return currentPose;
    }

    public Pose2d addVisionMeasurment(Pose2d robotPoseFromVision, double visionDataTimestamp) {
        poseEstimator.addVisionMeasurement(robotPoseFromVision, visionDataTimestamp);
        currentPose = poseEstimator.getEstimatedPosition();
        return currentPose;
    }

    public void changeVisionDevs(double visionStrdDevX, double visionStrdDevY, double visionStrdDevTheta) {
        poseEstimator.setVisionMeasurementStdDevs(
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStrdDevX, visionStrdDevY, visionStrdDevTheta));
    }

    public Pose2d updateWithVision(BreakerDiffDriveState currentDriveState, Pose2d robotPoseFromVision,
            double visionDataTimestamp) {
        update(currentDriveState);
        addVisionMeasurment(robotPoseFromVision, visionDataTimestamp);
        currentPose = poseEstimator.getEstimatedPosition();
        return currentPose;
    }

    public void setOdometryPosition(Pose2d newPose) {
        poseEstimator.resetPosition(newPose, Rotation2d.fromDegrees(gyro.getRawYaw()));
    }

    @Override
    public String toString() {
        return "CURRENT POSE: " + getOdometryPoseMeters().toString();
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        currentPose = poseEstimator.getEstimatedPosition();
        return currentPose;
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return curMovementState;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeChassisSpeeds.vxMetersPerSecond,
                fieldRelativeChassisSpeeds.vyMetersPerSecond,
                fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
                getOdometryPoseMeters().getRotation());
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return fieldRelativeChassisSpeeds;
    }

    private void updateChassisSpeeds() {
        double timeDiff = Timer.getFPGATimestamp() - lastUpdateTimestamp;
        double xSpeed = (getOdometryPoseMeters().getX() - prevPose.getX()) * timeDiff;
        double ySpeed = (getOdometryPoseMeters().getY() - prevPose.getY()) * timeDiff;
        double thetaSpeed = (getOdometryPoseMeters().getRotation().getRadians() - prevPose.getRotation().getRadians())
                * timeDiff;
        fieldRelativeChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(),
                fieldRelativeChassisSpeeds, timeDiff, prevMovementState);
    }

}
