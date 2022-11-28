// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photon;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionPoseFilter;

/** WIP */
public class BreakerVision implements BreakerGenericOdometer {
    private BreakerFiducialPhotonTarget[] targets;
    private BreakerPhotonCamera[] cameras;
    private BreakerVisionPoseFilter poseFilter;
    private BreakerVisionOdometer odometer;
    public BreakerVision(double poseFilterTrustCoef, double poseFilterMaxUncertanty, BreakerPhotonCamera[] cameras, Pair<Integer, Pose3d>[] fiducialTargetIDsAndPoses) {
        targets = new BreakerFiducialPhotonTarget[fiducialTargetIDsAndPoses.length];
        this.cameras = cameras;
        for (int i = 0; i < fiducialTargetIDsAndPoses.length; i++) {
            Pair<Integer, Pose3d> dataPair = fiducialTargetIDsAndPoses[i];
            targets[i] = new BreakerFiducialPhotonTarget(dataPair.getFirst(), dataPair.getSecond(), cameras);
        }

        poseFilter = new BreakerVisionPoseFilter(poseFilterTrustCoef, poseFilterMaxUncertanty, targets);
        odometer = new BreakerVisionOdometer(poseFilter);
    }

    public BreakerPhotonCamera[] getCameras() {
        return cameras;
    }

    public BreakerPhotonCamera getCamera(String cameraName) {
        for (BreakerPhotonCamera cam: cameras) {
            if (cam.getDeviceName() == cameraName) {
                return cam;
            }
        }
        return null;
    }

    public boolean hasTargets() {
        for (BreakerPhotonCamera cam: cameras) {
            if (cam.hasTargets()) {
                return true;
            }
        }
        return false;
    }

    public BreakerFiducialPhotonTarget[] getFiducialTargets() {
        return targets;
    }

    public Pose3d getFilteredRobotPose3d() {
        return poseFilter.getFilteredRobotPose3d();
    }

    public Pose2d getFilteredRobotPose() {
        return poseFilter.getFilteredRobotPose();
    }

    public BreakerVisionOdometer getBaseVisionOdometer() {
        return odometer;
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        odometer.setOdometryPosition(newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return odometer.getOdometryPoseMeters();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return odometer.getMovementState();
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return odometer.getRobotRelativeChassisSpeeds();
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return odometer.getFieldRelativeChassisSpeeds();
    }
}
