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

    
    /** 
     * @return BreakerPhotonCamera[]
     */
    public BreakerPhotonCamera[] getCameras() {
        return cameras;
    }

    
    /** 
     * @param cameraName
     * @return BreakerPhotonCamera
     */
    public BreakerPhotonCamera getCamera(String cameraName) {
        for (BreakerPhotonCamera cam: cameras) {
            if (cam.getDeviceName() == cameraName) {
                return cam;
            }
        }
        return null;
    }

    
    /** 
     * @return boolean
     */
    public boolean hasTargets() {
        for (BreakerPhotonCamera cam: cameras) {
            if (cam.hasTargets()) {
                return true;
            }
        }
        return false;
    }

    
    /** 
     * @return BreakerFiducialPhotonTarget[]
     */
    public BreakerFiducialPhotonTarget[] getFiducialTargets() {
        return targets;
    }

    
    /** 
     * @return Pose3d
     */
    public Pose3d getFilteredRobotPose3d() {
        return poseFilter.getFilteredRobotPose3d();
    }

    
    /** 
     * @return Pose2d
     */
    public Pose2d getFilteredRobotPose() {
        return poseFilter.getFilteredRobotPose();
    }

    
    /** 
     * @return BreakerVisionOdometer
     */
    public BreakerVisionOdometer getBaseVisionOdometer() {
        return odometer;
    }

    
    /** 
     * @param newPose
     */
    @Override
    public void setOdometryPosition(Pose2d newPose) {
        odometer.setOdometryPosition(newPose);
    }

    
    /** 
     * @return Pose2d
     */
    @Override
    public Pose2d getOdometryPoseMeters() {
        return odometer.getOdometryPoseMeters();
    }

    
    /** 
     * @return BreakerMovementState2d
     */
    @Override
    public BreakerMovementState2d getMovementState() {
        return odometer.getMovementState();
    }

    
    /** 
     * @return ChassisSpeeds
     */
    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return odometer.getRobotRelativeChassisSpeeds();
    }

    
    /** 
     * @return ChassisSpeeds
     */
    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return odometer.getFieldRelativeChassisSpeeds();
    }
}
