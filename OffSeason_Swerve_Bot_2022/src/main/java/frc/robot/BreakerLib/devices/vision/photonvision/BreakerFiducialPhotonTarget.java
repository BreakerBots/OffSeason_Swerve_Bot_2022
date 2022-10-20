// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photonvision;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class BreakerFiducialPhotonTarget extends SubsystemBase {
    private PhotonTrackedTarget assignedTarget;
    private double lastDataUpdate = Timer.getFPGATimestamp();

    private BreakerPhotonCamera camera;
    private BreakerPhotonCamera[] cameras;
    private Pose3d targetPose;
    private boolean assignedTargetFound = false;
    private boolean assignedTargetFoundInCycle = false; 
    private int fiducicalID;

    /**
     * 
     * @param fiducicalID - The numeric id of the Fidicial Marker, this is tied to the tag's pattern and is constant
     * @param targetPose - the 3 dimentional orientation of the target relative the the field origin
     * @param cameras - Any and all cameras that will be used to track the target
     */
    public BreakerFiducialPhotonTarget(int fiducicalID, Pose3d targetPose, BreakerPhotonCamera... cameras) {
        this.targetPose = targetPose;
        this.fiducicalID = fiducicalID;
        this.cameras = cameras;
    }

    private void findAssignedFiducial() {
        assignedTargetFoundInCycle = false;
        for (BreakerPhotonCamera cam: cameras) {
            if (cam.hasTargets()) {
                for (PhotonTrackedTarget prospTgt: cam.getAllRawTrackedTargets()) {
                    if (prospTgt.getFiducialId() == fiducicalID) {
                        assignedTarget = prospTgt;
                        assignedTargetFound = true;
                        assignedTargetFoundInCycle = true;
                        lastDataUpdate = Timer.getFPGATimestamp();
                        camera = cam;
                    }
                }
            }
        }
    }

    public double getLastTargetFooundTimestamp() {
        return lastDataUpdate;
    }

    public double getTargetDataAge() {
        double timediffsec = Timer.getFPGATimestamp() - lastDataUpdate;
        return Units.millisecondsToSeconds(camera.getPipelineLatancyMilliseconds()) + timediffsec;
    }

    public Pose3d getCameraPose3d() {
        return targetPose.transformBy(assignedTarget.getCameraToTarget().inverse());
    }

    public Pose2d getCameraPose() {
        return targetPose.transformBy(assignedTarget.getCameraToTarget().inverse()).toPose2d();
    }

    public Pose3d getRobotPose3d() {
        return getCameraPose3d().transformBy(camera.get3dCamPositionRelativeToRobot().inverse());
    }

    public Pose2d getRobotPose() {
        return getRobotPose3d().toPose2d();
    }

    /** Assigned target camera relative yaw */
    public double getYaw() {
        return assignedTarget.getYaw();
    }

    /** Assigned target camera relative pitch */
    public double getPitch() {
        return assignedTarget.getPitch();
    }
 
    public double getRobotRelativeYaw() {
        return Rotation2d.fromDegrees(getYaw()).minus(camera.getCamPositionRelativeToRobot().getRotation()).getDegrees();
    }

    public double getRobotRelativePitch() {
        return Rotation2d.fromDegrees(getYaw()).minus(new Rotation2d(camera.get3dCamPositionRelativeToRobot().getRotation().getY())).getDegrees();
    }

    /** Assigned target skew */
    public double getSkew() {
        return assignedTarget.getSkew();
    }

    /** Assigned target area */
    public double getArea() {
        return assignedTarget.getArea();
    }

    /** List of target corner coordinates. */
    public List<TargetCorner> getTargetCorners() {
        return assignedTarget.getCorners();
    }

    /** If assigned target has been found at any point during operation */
    public boolean getAssignedTargetFound() {
        return assignedTargetFound;
    }

    /** If assigned target was found in the most recent cycle */
    public boolean isAssignedTargetVisable() {
        return assignedTargetFound && assignedTargetFoundInCycle;
    }

    public double getDistance() {
        return assignedTarget.getCameraToTarget().getTranslation().toTranslation2d().getNorm();
    }

    public int getFiducialID() {
        return fiducicalID;
    }

    public double getPoseAmbiguity() {
        return assignedTarget.getPoseAmbiguity();
    }

    @Override
    public void periodic() {
        findAssignedFiducial();
    }
}
